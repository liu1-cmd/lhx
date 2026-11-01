#include "tracker.h"
#include "linefollower.h"
#include "pid.h"

/* ==================================================================
 * 循迹策略（简化版）
 *
 *   RUN    : PD 循迹。误差大就减速，盲区(全白/全黑)限速"摸着走"。
 *   PIVOT  : 丢线超过 100ms，原地转找线。朝"最后看到线的方向"转 1.6s，
 *            没找到就反向 0.6s（转回去一半），再朝原方向 1.6s……
 *            每轮净转 1s，相当于一圈圈扫过去，总能扫到线，且永不"停车"。
 *
 * 丢掉的东西：卡死脱困状态机、误差/掩码恒定检测、停车保护。
 * 这三个都是为了处理"车卡住不动"，但都会让车自己停下来。若以后又出现
 * 堵转，再按需加，别让策略体积先涨上去。
 * ================================================================== */

/* ---------------- 可调参数 ---------------- */
#define BASE_MAX      2200    /* 直线基础速度（3200 为满占空比），可用 base= 在线改 */
#define BASE_MIN       700    /* 急弯最低速度 */
#define SLOW_K         150    /* 每档误差的减速量：err=±7 时降到 BASE_MIN */
#define BLIND_SPEED   1100    /* 全白/全黑盲区限速：8 路都给不出误差，别全速冲 */

#define LOST_LIMIT      20    /* 连续全白周期数(5ms×20=100ms) 才认为真丢线。
                               线常落进 X4/X5 死区，阈值太小会疯狂误触 pivot */
#define PIVOT_SPEED   2400    /* 原地转单轮占空比：低电压/搓地阻力下也要转得动 */
#define PIVOT_TIMEOUT  320    /* 首次朝一侧转 1.6s */
#define PIVOT_RETRY    120    /* 反向回扫 0.6s：比去程短，形成净前进的扫掠 */

#define CTRL_DT      0.005f   /* 控制周期，由 empty.c 的 SysTick 5ms 保证 */
#define PID_LIMIT   2000.0f   /* PID 输出限幅 */

/* 基础速度斜坡：误差每跨一个探头跳 2 档，目标速度跟着跳 300，
 * 直接下发就是"加速-急刹"的顿挫。加速慢、减速快。 */
#define SLEW_UP       60
#define SLEW_DOWN    120

/* LF_Read 位：bit7=X1(最左) ... bit0=X8(最右) */
#define MASK_X1   0x80
#define MASK_X8   0x01
#define MASK_MID  0x18        /* X4 X5 中间两路 */

/* ---------------- 在线可调参数 ---------------- */
static float   g_kp = 180.0f;
static float   g_ki = 0.0f;
static float   g_kd = 0.8f;
static float   g_tau = 0.01f;      /* 误差一阶低通时间常数，0=关闭 */
static int16_t g_base_max = BASE_MAX;
static uint8_t g_outer_pivot = 0;  /* 最外侧探头压线是否直接 pivot，默认关 */

/* ---------------- 内部状态 ---------------- */
typedef enum { ST_RUN = 0, ST_PIVOT = 1 } TrkState;

static PID      steer;
static TrkState state     = ST_RUN;
static int8_t   last_side = 0;     /* 最近一次明确的偏离方向：-1 线在左, +1 线在右 */
static int8_t   pivot_dir = 0;
static uint8_t  pivot_back = 0;    /* 0=正在去程(1.6s) 1=正在回扫(0.6s) */
static uint16_t lost_cnt  = 0;
static uint16_t pivot_cnt = 0;
static uint8_t  outer_cnt = 0;
static float    err_f     = 0.0f;  /* 滤波后的连续误差，只喂 PD */
static int16_t  base_ramp = 0;     /* 斜坡后的基础速度，0 起步 */

static int16_t  cur_err   = 0;
static uint8_t  cur_line  = 0;
static int16_t  cur_dutyA = 0;
static int16_t  cur_dutyB = 0;

static void pid_reset(void)
{
    PID_Init(&steer, g_kp, g_ki, g_kd, -PID_LIMIT, PID_LIMIT, 100.0f);
}

/* 一阶低通：把离散的 -7,-5,…,+7 误差变成连续量，PD 输出才不会跳台阶 */
static float err_filter(float x)
{
    if (g_tau <= 0.0f) { err_f = x; return x; }
    err_f += (CTRL_DT / (g_tau + CTRL_DT)) * (x - err_f);
    return err_f;
}

static int16_t base_slew(int16_t target)
{
    int16_t d = (int16_t)(target - base_ramp);
    if (d >  SLEW_UP)   d =  SLEW_UP;
    if (d < -SLEW_DOWN) d = -SLEW_DOWN;
    base_ramp = (int16_t)(base_ramp + d);
    return base_ramp;
}

/* 进入 PIVOT：朝最后看到线的方向转 */
static void pivot_enter(int8_t dir)
{
    state      = ST_PIVOT;
    pivot_dir  = dir;
    pivot_back = 0;
    pivot_cnt  = 0;
    pid_reset();
}

/* 退出 PIVOT：把滤波和斜坡对齐真实值，避免切回 PD 瞬间猛打方向 */
static void pivot_exit(int8_t err)
{
    state     = ST_RUN;
    lost_cnt  = 0;
    pivot_cnt = 0;
    base_ramp = BASE_MIN;
    pid_reset();
    steer.last_err = (float)err;
    err_f = (float)err;
}

/* 从遥控等外部模式切回循迹：状态归零 + 速度从 0 软启动。
 * 注意不重置在线调好的增益和 base（那是 Tracker_Init 的活）。 */
void Tracker_Resume(void)
{
    state     = ST_RUN;
    last_side = 0;
    pivot_dir = 0;
    pivot_back = 0;
    lost_cnt  = 0;
    pivot_cnt = 0;
    outer_cnt = 0;
    base_ramp = 0;
    err_f     = 0.0f;
    pid_reset();
}

/* ---------------- 调参接口（保持与 tuner.c 一致） ---------------- */
void Tracker_SetKP(float kp) { g_kp = kp; steer.kp = kp; }
void Tracker_SetKI(float ki) { g_ki = ki; steer.ki = ki; }
void Tracker_SetKD(float kd) { g_kd = kd; steer.kd = kd; }
void Tracker_SetOuterPivot(uint8_t on) { g_outer_pivot = on ? 1 : 0; }
uint8_t Tracker_OuterPivot(void)       { return g_outer_pivot; }

void Tracker_SetTau(float tau)
{
    if (tau < 0.0f) tau = 0.0f;
    if (tau > 0.5f) tau = 0.5f;
    g_tau = tau;
}

void Tracker_SetBase(float duty)
{
    if (duty < 500.0f)  duty = 500.0f;
    if (duty > 3200.0f) duty = 3200.0f;
    g_base_max = (int16_t)duty;
}

float Tracker_GetTau(void) { return g_tau; }
float Tracker_ErrFiltered(void) { return err_f; }

void Tracker_GetGains(float *kp, float *ki, float *kd)
{
    if (kp) *kp = g_kp;
    if (ki) *ki = g_ki;
    if (kd) *kd = g_kd;
}

void Tracker_GetDebug(int16_t *err, float *p, float *i, float *d,
                      int16_t *dutyA, int16_t *dutyB)
{
    *err   = cur_err;
    *p     = steer.last_p;
    *i     = steer.last_i;
    *d     = steer.last_d;
    *dutyA = cur_dutyA;
    *dutyB = cur_dutyB;
}

void Tracker_Init(void)
{
    pid_reset();
    state      = ST_RUN;
    last_side  = 0;
    pivot_dir  = 0;
    pivot_back = 0;
    lost_cnt   = 0;
    pivot_cnt  = 0;
    outer_cnt  = 0;
    cur_err    = 0;
    cur_line   = 0;
    err_f      = 0.0f;
    base_ramp  = 0;
    g_base_max = BASE_MAX;
}

uint8_t Tracker_State(void) { return (uint8_t)state; }
int8_t  Tracker_Err(void)   { return cur_err; }
uint8_t Tracker_Line(void)  { return cur_line; }

/* ---------------- 主循环，5ms 调用一次 ---------------- */
void Tracker_Update(int16_t *dutyA, int16_t *dutyB)
{
    uint8_t line = LF_Read();
    int8_t  err  = LF_ErrorFrom(line);          /* 同一次采样，line/err 不会错位 */
    float   ferr = err_filter((float)err);

    cur_err  = err;
    cur_line = line;

    if (state == ST_RUN) {

        /* 1. 记住最后看到线的方向。只在 |err|>=2 或贴最外侧时才更新：
         *    ±1 是 X4/X5 中心抖动，拿它定转向等于掷硬币。
         *    直角弯全黑时 err=0 没有左右信息，丢线后要沿用这里的方向。 */
        if (err <= -2) last_side = -1;
        else if (err >= 2) last_side = 1;
        if ((line & MASK_X1) && !(line & MASK_X8)) last_side = -1;
        else if ((line & MASK_X8) && !(line & MASK_X1)) last_side = 1;

        /* 2. 连续全白 = 真丢线 → 原地找线 */
        if (line == 0x00) {
            if (++lost_cnt >= LOST_LIMIT) {
                pivot_enter((last_side != 0) ? last_side : -1);
            }
        } else {
            lost_cnt = 0;
        }

        /* 3. 可选：最外侧探头压线就是直角/锐角（中间两路没压，排除宽线误判），
         *    连续 4 个周期(20ms) 才触发，防颠簸误动作。默认关闭，交给 PD 打满差速。 */
        if (g_outer_pivot && state == ST_RUN && line != 0xFF &&
            (line & (MASK_X1 | MASK_X8)) && !(line & MASK_MID)) {
            if (++outer_cnt >= 4) {
                pivot_enter((line & MASK_X1) ? -1 : 1);
                outer_cnt = 0;
            }
        } else {
            outer_cnt = 0;
        }

        /* 4. PD 循迹
         *    减速用原始误差（弯道一出现立刻降速），转向用滤波后误差（输出平滑）；
         *    全白/全黑盲区给不出误差，限速"摸着走"，避免 err=0 时全速冲弯。 */
        if (state == ST_RUN) {
            int16_t aerr   = (err < 0) ? -err : err;
            int16_t target = (int16_t)(g_base_max - SLOW_K * aerr);
            if (target < BASE_MIN) target = BASE_MIN;
            if ((line == 0x00 || line == 0xFF) && target > BLIND_SPEED) {
                target = BLIND_SPEED;
            }

            int16_t base = base_slew(target);
            float   out  = PID_Compute(&steer, ferr, CTRL_DT);
            if (out >  (float)base) out =  (float)base;
            if (out < -(float)base) out = -(float)base;

            /* 差速极性：X1 侧=A 轮，X8 侧=B 轮；out>0 → A 慢 B 快，车拧向 B 侧。
             * 下面的 pivot 用同一套约定，改一处就要两处一起改。 */
            *dutyA = (int16_t)(base - (int16_t)out);
            *dutyB = (int16_t)(base + (int16_t)out);
        }
    }

    if (state == ST_PIVOT) {
        if (line != 0x00) {
            /* 任意探头见线就交回 PD。转圈时线往往先扫到外侧探头，
             * 等中间两路会把时间白耗掉。 */
            pivot_exit(err);
            *dutyA = BASE_MIN;
            *dutyB = BASE_MIN;
        } else {
            /* 去程 1.6s 找不到 → 反向回扫 0.6s → 再朝原方向 1.6s……
             * 每轮净转 1s，一圈圈扫过去总能扫到线，且不会停下来。 */
            uint16_t limit = pivot_back ? PIVOT_RETRY : PIVOT_TIMEOUT;
            if (++pivot_cnt >= limit) {
                pivot_cnt  = 0;
                pivot_back = pivot_back ? 0 : 1;
                pivot_dir  = (int8_t)(-pivot_dir);
                pid_reset();
            }
            if (pivot_dir < 0) {          /* dir<0：朝 X1 侧拧 → A 反转 B 正转 */
                *dutyA = -PIVOT_SPEED;
                *dutyB =  PIVOT_SPEED;
            } else {
                *dutyA =  PIVOT_SPEED;
                *dutyB = -PIVOT_SPEED;
            }
        }
    }

    cur_dutyA = *dutyA;
    cur_dutyB = *dutyB;
}
