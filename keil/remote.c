#include "remote.h"
#include "tracker.h"
#include "tuner.h"
#include "usart.h"
#include "motor.h"

#define SPEED_DEFAULT   1200    /* 遥控默认速度（3200 为满占空比） */
#define TIMEOUT_MS       800    /* 命令断流超时：超时即停车（防蓝牙掉线跑飞） */

enum { DIR_STOP = 0, DIR_FWD, DIR_BACK, DIR_LEFT, DIR_RIGHT };

static uint8_t  s_mode  = REMOTE_AUTO;
static uint8_t  s_dir   = DIR_STOP;
static uint16_t s_life  = 0;     /* 当前指令还剩多少毫秒有效 */
static int16_t  s_speed = SPEED_DEFAULT;

static char to_lower(char c)
{
    return (c >= 'A' && c <= 'Z') ? (char)(c + 32) : c;
}

/* 取开头的十进制整数，不做越界检查（上层会 clamp） */
static int parse_int(const char *p)
{
    int v = 0;
    int sign = 1;

    while (*p == ' ') p++;
    if (*p == '-') { sign = -1; p++; }
    while (*p >= '0' && *p <= '9') {
        v = v * 10 + (*p - '0');
        p++;
    }
    return sign * v;
}

/* 匹配 "key=值" / "key:值" / "key 值"（大小写不敏感、允许等号前后空格）。
 * 命中返回 1 并写 *out；否则返回 0。 */
static uint8_t key_match(const char *s, const char *key, int *out)
{
    int i = 0;

    while (key[i] != '\0') {
        if (to_lower(s[i]) != key[i]) return 0;
        i++;
    }
    /* 键名后必须紧跟分隔符，否则 "modulation" 之类会被误判成 mode */
    if (s[i] == '=' || s[i] == ':') {
        i++;
    } else if (s[i] == ' ' || s[i] == '\t') {
        while (s[i] == ' ' || s[i] == '\t') i++;
    } else {
        return 0;
    }

    *out = parse_int(s + i);
    return 1;
}

void Remote_Init(void)
{
    s_mode  = REMOTE_AUTO;
    s_dir   = DIR_STOP;
    s_life  = 0;
    s_speed = SPEED_DEFAULT;
}

uint8_t Remote_Mode(void) { return s_mode; }

static void set_mode(uint8_t manual)
{
    s_dir  = DIR_STOP;
    s_life = 0;
    if (manual) {
        s_mode = REMOTE_MANUAL;       /* 遥控期间循迹状态机不再跑，切回来时重启 */
        Motor_Set(0, 0);              /* 立刻刹停：不等下一个控制周期，车立定等遥控 */
    } else {
        s_mode = REMOTE_AUTO;
        Tracker_Resume();             /* 软启动，不保留遥控期间的状态 */
    }
    /* 只有开了回显才回 ACK：默认关，避免文本行污染波形数字流。
     * 调试时先发 echo=1，再发 mode=1，能看到 ACK 就证明命令确实被遥控模块吃到了 */
    if (Tuner_EchoEnabled()) {
        USART_SendString(manual ? "ACK:mode manual\r\n" : "ACK:mode auto\r\n");
    }
}

uint8_t Remote_HandleLine(const char *s)
{
    char c;
    int  v;

    if (s == 0) return 0;
    while (*s == ' ' || *s == '\t') s++;   /* 容忍行首空白（有的串口助手会自动加） */
    if (s[0] == '\0') return 0;

    /* ---- 遥控指令：单字符，或同一字符被重复堆叠 ----
     * 手机 App "按住连发"常常不带换行，会把 f 累积成 "ffff"，
     * 这里允许整行是同一个遥控字符的重复，取最后一个的有效方向。
     * 注意 b/m 开头还有 base= / mode= 等键名命令，所以只认整行纯重复才算。 */
    c = to_lower(s[0]);
    if (c == 'f' || c == 'b' || c == 'l' || c == 'r' || c == 'x') {
        int i = 1;
        while (s[i] != '\0' && to_lower(s[i]) == c) i++;
        if (s[i] == '\0') {
            switch (c) {
            case 'f': s_dir = DIR_FWD;   break;
            case 'b': s_dir = DIR_BACK;  break;
            case 'l': s_dir = DIR_LEFT;  break;
            case 'r': s_dir = DIR_RIGHT; break;
            default:  s_dir = DIR_STOP;  break;
            }
            s_life = TIMEOUT_MS;   /* 刷新有效期，持续发就一直走 */
            return 1;
        }
    }

    /* ---- 简写：m1 进入遥控 / m0 回到循迹 ---- */
    if (s[1] != '\0' && s[2] == '\0' && to_lower(s[0]) == 'm' && (s[1] == '1' || s[1] == '0')) {
        set_mode((uint8_t)(s[1] == '1'));
        return 1;
    }

    /* ---- mode=0/1：自动循迹 / 手动遥控，接受 = : 或空格分隔 ---- */
    if (key_match(s, "mode", &v)) {
        set_mode(v != 0);
        return 1;
    }

    /* ---- spd=300~3200：遥控速度 ---- */
    if (key_match(s, "spd", &v)) {
        if (v < 300)  v = 300;
        if (v > 3200) v = 3200;
        s_speed = (int16_t)v;
        return 1;
    }

    return 0;
}

void Remote_Update(int16_t *dutyA, int16_t *dutyB)
{
    /* 指令寿命按 5ms 递减；断流后自动转成停车 */
    if (s_life >= 5) s_life = (uint16_t)(s_life - 5);
    else             s_life = 0;
    if (s_life == 0) s_dir = DIR_STOP;

    switch (s_dir) {
    case DIR_FWD:                                   /* 前进：两轮正转 */
        *dutyA = s_speed;
        *dutyB = s_speed;
        break;
    case DIR_BACK:                                  /* 后退：两轮反转 */
        *dutyA = -s_speed;
        *dutyB = -s_speed;
        break;
    case DIR_LEFT:                                  /* 左转：右轮正转、左轮反转 */
        *dutyA = -s_speed;
        *dutyB =  s_speed;
        break;
    case DIR_RIGHT:                                 /* 右转：左轮正转、右轮反转 */
        *dutyA =  s_speed;
        *dutyB = -s_speed;
        break;
    default:                                        /* 停车 */
        *dutyA = 0;
        *dutyB = 0;
        break;
    }
}
