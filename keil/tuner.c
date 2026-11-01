#include "tuner.h"
#include "usart.h"
#include "tracker.h"
#include "remote.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 手工整数转字符串：工程用的是标准库（非 MicroLIB），
 * snprintf 会把整个 printf 引擎链进来，栈开销几百字节；
 * 而原 Stack_Size 只有 256B，第一帧就会踩栈导致 HardFault
 *（现象：复位能看到 BOOT 横幅，但周期遥测帧一帧都没有）。
 * 这里零库依赖格式化，顺带省掉一大块 ROM。 */
static char *put_int(char *p, int v)
{
    char tmp[8];
    uint8_t n = 0;
    uint32_t u;

    if (v < 0) { *p++ = '-'; u = (uint32_t)(-(int32_t)v); }
    else       {             u = (uint32_t)v; }

    do {
        tmp[n++] = (char)('0' + (u % 10u));
        u /= 10u;
    } while (u && n < sizeof(tmp));

    while (n) *p++ = tmp[--n];
    return p;
}

/* 浮点不直接发 %f，统一放大 10 倍发整数 */
void VOFA_SendFrame(void)
{
    int16_t err, dA, dB;
    float p, i, d, kp, ki, kd;
    char buf[80];
    char *q;

    Tracker_GetDebug(&err, &p, &i, &d, &dA, &dB);
    Tracker_GetGains(&kp, &ki, &kd);

    q = put_int(buf,   (int)err);            *q++ = ',';   /* 原始误差（台阶） */
    q = put_int(q,     (int)(p * 10.0f));    *q++ = ',';
    /* ch2 改为滤波后的连续误差 ×10（原 I 项在 ki=0 时恒为 0，没信息量） */
    q = put_int(q,     (int)(Tracker_ErrFiltered() * 10.0f)); *q++ = ',';
    q = put_int(q,     (int)(d * 10.0f));    *q++ = ',';
    q = put_int(q,     (int)dA);             *q++ = ',';
    q = put_int(q,     (int)dB);             *q++ = ',';
    /* ch6：0=循迹 1=原地找线 2=蓝牙遥控模式。
     * 遥控是否真的切进去了，在波形上直接看这一路，不用猜。 */
    q = put_int(q,     (Remote_Mode() == REMOTE_MANUAL) ? 2 : (int)Tracker_State());
    *q++ = ',';
    q = put_int(q,     (int)(kp * 10.0f));   *q++ = ',';
    q = put_int(q,     (int)(ki * 10.0f));   *q++ = ',';
    q = put_int(q,     (int)(kd * 10.0f));   *q++ = ',';
    /* ch10: 原始 8 位亮灯掩码 0~255（bit7=X1最左 ... bit0=X8最右），与 ch0 同采样 */
    q = put_int(q,     (int)Tracker_Line());
    *q++ = '\r';
    *q++ = '\n';
    *q   = '\0';

    USART_SendString(buf);
}

/* 行缓冲：收到 \r 或 \n 视为一条命令结束；
 * 手机蓝牙串口助手常常不带换行，所以再加一条"静默超时"规则。 */
static char    line[24];
static uint8_t line_len = 0;

#define LINE_IDLE_TICKS  10u        /* 10 × 5ms = 50ms 无新字节即认为一行结束 */
static uint32_t line_tick = 0;
extern volatile uint32_t g_ctrl_ticks;   /* empty.c 里的 5ms 控制周期计数 */

/* 大小写不敏感比较两个字符 */
static int ci_eq(char a, char b)
{
    if (a >= 'A' && a <= 'Z') a += 32;
    if (b >= 'A' && b <= 'Z') b += 32;
    return a == b;
}

/* 回显开关：默认关闭。
 * ACK/ERR 文本行混进 FireWater 数字流会让 VOFA+ 波形解析卡住（"收不到实时数据"的元凶）。
 * 需要调试发送格式时，发 "echo=1" 打开回显，"echo=0" 关闭。 */
static uint8_t g_echo = 0;

/* OLED 刷新开关：软件 I2C 刷屏会把控制周期打断十几毫秒，
 * 正式跑圈时发 "oled=0" 关掉，只看 VOFA 波形即可。 */
static uint8_t g_oled = 1;

uint8_t Tuner_OLEDEnabled(void) { return g_oled; }
uint8_t Tuner_EchoEnabled(void) { return g_echo; }   /* 遥控模块用它决定是否回 ACK */

static void echo_back(const char *tag, const char *s)
{
    if (!g_echo) return;
    USART_SendString(tag);
    USART_SendString(s);
    USART_SendString("\r\n");
}

/* 宽容解析：忽略行首空白；分隔号同时接受 '=' 和 ':'；大小写不敏感。
 * 支持：kp=200 / kd:5.4 / KI=0.1 / echo=1 等写法 */
static void handle_command(char *s)
{
    char *p = s;
    char *sep;
    double v;

    while (*p == ' ' || *p == '\t') p++;   /* 跳过行首空白 */
    if (*p == '\0') return;                /* 空行忽略 */

    /* 找第一个 '=' 或 ':' 作为分隔符 */
    sep = p;
    while (*sep != '\0' && *sep != '=' && *sep != ':') sep++;
    if (*sep == '\0') { echo_back("ERR:", p); return; }

    /* echo 开关命令：必须精确匹配 "echo"。
     * 原规则 p[0]=='e' 且长度>=2 会把链路损坏的 "ei=0.64"（ki 的 k 被打成 e）
     * 误判为 echo 命令，意外打开回显，ERR 文本污染波形流。 */
    if (sep - p == 4 && ci_eq(p[0], 'e') && ci_eq(p[1], 'c') &&
        ci_eq(p[2], 'h') && ci_eq(p[3], 'o')) {
        v = strtod(sep + 1, NULL);
        g_echo = (v >= 0.5f) ? 1 : 0;
        if (g_echo) USART_SendString("ACK:echo on\r\n");
        return;
    }

    /* OLED 刷新开关："oled=0/1" */
    if (sep - p == 4 && ci_eq(p[0], 'o') && ci_eq(p[1], 'l') &&
        ci_eq(p[2], 'e') && ci_eq(p[3], 'd')) {
        v = strtod(sep + 1, NULL);
        g_oled = (v >= 0.5) ? 1 : 0;
        echo_back("ACK:", p);
        return;
    }

    /* 基础速度在线调整："base=1800"（500~3200） */
    if (sep - p == 4 && ci_eq(p[0], 'b') && ci_eq(p[1], 'a') &&
        ci_eq(p[2], 's') && ci_eq(p[3], 'e')) {
        v = strtod(sep + 1, NULL);
        if (v < 500.0 || v > 3200.0) { echo_back("ERR:range ", p); return; }
        Tracker_SetBase((float)v);
        echo_back("ACK:", p);
        return;
    }

    /* 外侧探头 pivot 触发开关："pivot=0/1"（0=关，默认；1=旧行为） */
    if (sep - p == 5 && ci_eq(p[0], 'p') && ci_eq(p[1], 'i') &&
        ci_eq(p[2], 'v') && ci_eq(p[3], 'o') && ci_eq(p[4], 't')) {
        v = strtod(sep + 1, NULL);
        Tracker_SetOuterPivot(v >= 0.5 ? 1 : 0);
        echo_back("ACK:", p);
        return;
    }

    /* 滤波时间常数命令："tau=0.04"（秒）；0=关闭低通，上限 0.5 */
    if (sep - p == 3 && ci_eq(p[0], 't') && ci_eq(p[1], 'a') && ci_eq(p[2], 'u')) {
        v = strtod(sep + 1, NULL);
        if (v < 0.0 || v > 0.5) { echo_back("ERR:range ", p); return; }
        Tracker_SetTau((float)v);
        echo_back("ACK:", p);
        return;
    }

    /* 增益命令：k + p/i/d */
    if (!(ci_eq(p[0], 'k') && (ci_eq(p[1], 'p') || ci_eq(p[1], 'i') || ci_eq(p[1], 'd')))) {
        echo_back("ERR:", p);
        return;
    }

    v = strtod(sep + 1, NULL);             /* strtod 自动跳过数字前的空格 */

    /* 范围校验：拒绝链路损坏/乱码产生的异常值，防止写炸增益 */
    if (v < 0.0 || v > 1000.0) { echo_back("ERR:range ", p); return; }

    if      (ci_eq(p[1], 'p')) Tracker_SetKP((float)v);
    else if (ci_eq(p[1], 'i')) Tracker_SetKI((float)v);
    else                       Tracker_SetKD((float)v);

    echo_back("ACK:", p);
}

/* 提交缓冲区里的一行：去掉行尾空白后先给遥控模块，再给调参模块 */
static void submit_line(void)
{
    while (line_len > 0 && (line[line_len - 1] == ' ' || line[line_len - 1] == '\t')) {
        line_len--;                    /* 手机键盘容易多带一个尾随空格 */
    }
    if (line_len > 0) {
        line[line_len] = '\0';
        if (!Remote_HandleLine(line)) handle_command(line);
    }
    line_len = 0;
}

void VOFA_Poll(void)
{
    int16_t c;
    while ((c = USART_GetChar()) >= 0) {
        line_tick = g_ctrl_ticks;      /* 记录最近一次收到字节的时刻 */
        if (c == '\r' || c == '\n') {
            submit_line();
        } else if (c < 0x20 || c > 0x7E) {
            line_len = 0;   /* 不可打印字节=链路噪声，丢弃当前残行，
                             * 避免乱码行触发 ERR 回显污染数字流 */
        } else if (line_len < sizeof(line) - 1) {
            line[line_len++] = (char)c;
        } else {
            /* 缓冲区满：先按一整行处理。手机"按住连发"会把同一个字符
             * 堆到超过 24 字节，直接丢弃就等于按键失灵，必须提交。 */
            submit_line();
        }
    }

    /* 手机 App 不发换行时：一行字符静默 50ms 就当作"发送完毕"。
     * 这样 f / mode=1 无论带不带 \r\n 都能生效。
     * 50ms 的取值保证正常逐字节串行发送的 "kp=180" 不会在中途被截断。 */
    if (line_len > 0 && (uint32_t)(g_ctrl_ticks - line_tick) >= LINE_IDLE_TICKS) {
        submit_line();
    }
}
