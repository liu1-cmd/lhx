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
#define BASE_MAX      1400    /* 直线基础速度（3200 为满占空比），可用 base= 在线改 */
#define BASE_MIN       550    /* 急弯最低速度 */
#define SLOW_K         150    /* 每档误差的减速量：err=±7 时降到 BASE_MIN */
#define BLIND_SPEED    850    /* 全白/全黑盲区限速：8 路都给不出误差，别全速冲 */

#define LOST_LIMIT      20    /* 连续全白周期数(5ms×20=100ms) 才认为真丢线。
                               线常落进 X4/X5 死区，阈值太小会疯狂误触 pivot */
#define PIVOT_SPEED   1600    /* 原地转单轮占空比：50%。2400(75%) 太猛，先从中速试 */
#define PIVOT_TIMEOUT  320    /* 首次朝一侧转 1.6s */
#define PIVOT_RETRY    120    /* 反向回扫 0.6s：比去程短，形成净前进的扫掠 */

/* 原地转的差速极性：+1 / -1。
 * 这个符号只影响 pivot 的两个轮子谁正谁反，和 PD 的 dutyA/dutyB 无关。
 * 台架上如果发现"该往左拧却往右拧"，把这里翻一下就行，不用动别处。 */
#define PIVOT_SIGN     (-1)

/* 锐角原地转的"看线"参数。车是绕线旋转的：拧到 90° 时新线正好压住中间两路，
 * 再转下去线就从另一侧扫出去了，届时"看到线"就退会让车顺着原路反着走
 * （一圈走成倒着跑）。所以窗口内也要盯，对准了就立刻收手。 */
#define PIVOT_MIN_TURN   12   /* 先转 60ms 再认"对准"，防止进场那两拍就退 */
#define PIVOT_ACUTE_MAX  80   /* 400ms 还对不准就放宽判据，别一直自转 */
#define PIVOT_ACUTE_TURN 140  /* 锐角盲转硬上限 700ms：转够就强制交回 PD。
                               * 锐角时原主线一直压在阵列里，反向回扫必然扫回
                               * 原线 → 退出后车头对着来路 → 顺着原路倒着跑。
                               * 所以锐角只朝一个方向转，绝不换向。 */

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
#define MASK_L_OUT  0xC0u     /* X1 X2 左侧外探头 */
#define MASK_R_OUT  0x03u     /* X7 X8 右侧外探头 */
#define MASK_L_GAP  0x60u     /* X2 X3：X1 与 X4/X5 之间 */
#define MASK_R_GAP  0x06u     /* X6 X7：X5 与 X8 之间 */

/* ---------------- 在线可调参数 ---------------- */
static float   g_kp = 180.0f;
static float   g_ki = 0.0f;
static float   g_kd = 0.8f;
static float   g_tau = 0.01f;      /* 误差一阶低通时间常数，0=关闭 */
static int16_t g_base_max = BASE_MAX;
static uint8_t g_outer_pivot = 1;  /* 锐角检测默认开，pivot=0 可在线关 */

/* ---------------- 内部状态 ---------------- */
typedef enum { ST_RUN = 0, ST_PIVOT = 1 } TrkState;

static PID      steer;
static TrkState state     = ST_RUN;
static int8_t   last_side = 0;     /* 最近一次明确的偏离方向：-1 线在左, +1 线在右 */
static int8_t   pivot_dir = 0;
static uint8_t  pivot_back = 0;    /* 0=正在去程(1.6s) 1=正在回扫(0.6s) */
static uint8_t  pivot_hold = 0;    /* 进入后强制盲转的周期数：锐角进弯时必须>0 */
static uint8_t  pivot_need_mid = 0;/* 1=锐角进弯：线回到中间 X4/X5 才交回 PD，
                                      外侧扫到就退会把弯放掉，转一半→丢线→再转 */
static uint8_t  mid_ok    = 0;     /* 锐角盲转窗内"对准新线"连续成立的拍数 */
static uint8_t  pivot_acute = 0;   /* 1=本次 pivot 由锐角检测触发：只朝一个方向转，禁止反向回扫 */
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

/* 进入 PIVOT：朝最后看到线的方向转
 * hold：进入后强制盲转的周期数（5ms/周期）。
 *   丢线进入填 0 —— 此时 line 全白，靠"任意探头见线"退出没有问题；
 *   锐角进入必须填几十拍 —— 此时 line 本来就非 0，
 *   不盲转就会在同一拍被判成"已经找到线"立刻退出，pivot 等于没做。
 * need_mid：1=只有中间 X4/X5 见线才退出（锐角进弯用），
 *   否则支线刚从外侧探头扫进来就交回 PD，锐角永远转不完。 */
static void pivot_enter(int8_t dir, uint8_t hold, uint8_t need_mid,
                        uint8_t acute)
{
    state          = ST_PIVOT;
    pivot_dir      = dir;
    pivot_back     = 0;
    pivot_cnt      = 0;
    pivot_hold     = hold;
    pivot_need_mid = need_mid;
    pivot_acute    = acute;
    mid_ok         = 0;
    pid_reset();
}

/* 退出 PIVOT：把滤波和斜坡对齐真实值，避免切回 PD 瞬间猛打方向 */
static void pivot_exit(int8_t err)
{
    state     = ST_RUN;
    lost_cnt  = 0;
    pivot_cnt = 0;
    pivot_hold = 0;
    pivot_need_mid = 0;
    mid_ok = 0;
    pivot_acute = 0;
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
    pivot_hold = 0;
    pivot_need_mid = 0;
    pivot_acute = 0;
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
    pivot_hold = 0;
    pivot_need_mid = 0;
    pivot_acute = 0;
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
                pivot_enter((last_side != 0) ? last_side : -1, 0, 0, 0);
            }
        } else {
            lost_cnt = 0;
        }

        /* 3. 锐角直接 pivot（pivot=0 可关）：一侧最外探头(X1/X8)压线、
         *    中间 X4/X5 也压线、对侧外探头没压 —— 说明进弯直线和出弯支线
         *    同时在阵列下，是锐角/发夹特征。旧判据要求 X3/X6 必须熄灭
         *    （精确 1245/4578），实际进弯常扫成 12345/124，永不触发。
         *    连续 4 拍才信。 */
        if (g_outer_pivot && line != 0xFF) {
            /* 必须是"两段线"：最外探头亮、中间也亮、且两者之间至少灭一路。
             * 一段粗线/斜线也能扫出 0xF8(X1..X5) 这种连片图案，只看"外+中"
             * 会把它当成发夹去转，而车本来正跟着这条线走 —— 实测 40 次
             * pivot 里大半是这种误判，转完就丢线。 */
            uint8_t acute_l = (line & MASK_X1) && (line & MASK_MID) &&
                              !(line & MASK_R_OUT) &&
                              ((line & MASK_L_GAP) != MASK_L_GAP);
            uint8_t acute_r = (line & MASK_X8) && (line & MASK_MID) &&
                              !(line & MASK_L_OUT) &&
                              ((line & MASK_R_GAP) != MASK_R_GAP);
            if (acute_l || acute_r) {
                if (++outer_cnt >= 4) {
                    pivot_enter(acute_l ? -1 : 1, 30, 1, 1); /* 150ms 兜底；线一对准就提前退 */
                    outer_cnt = 0;
                }
            } else {
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

            /* 差速极性（台架实测）：A 轮=物理左=X1 侧，B 轮=物理右=X8 侧；
             * 车头永远朝慢轮一侧拧。out>0(线在右) → B 慢 A 快 → 向右拧向线。 */
            *dutyA = (int16_t)(base - (int16_t)out);
            *dutyB = (int16_t)(base + (int16_t)out);
        }
    }

    if (state == ST_PIVOT) {
        uint8_t exited   = 0;
        /* 对准判据：中间两路都压着、外侧四路全灭 —— 车头正对新线 */
        uint8_t lined_up = ((line & MASK_MID) == MASK_MID) &&
                           !(line & (MASK_L_OUT | MASK_R_OUT));
        /* 线在中间区：X4/X5 压住任意一路、外侧四路全灭。
         * 细线常只压中间一路，丢线退出用这个稍宽判据 */
        uint8_t mid_zone = (line & MASK_MID) &&
                           !(line & (MASK_L_OUT | MASK_R_OUT));

        if (pivot_hold) {
            /* 锐角进弯的盲转窗。窗口里 line 本来就非 0，不能拿"看到线"当退出
             * 条件（进弯当拍就会退，等于没转）；但也不能死等到窗口结束：车是
             * 绕线旋转的，拧过 90° 后新线会从另一侧扫回中间，那时再退就是顺着
             * 原路反着走了。所以这里盯"对准"，立刻收手。 */
            if (pivot_need_mid && pivot_cnt >= PIVOT_MIN_TURN && lined_up) {
                if (++mid_ok >= 2) {
                    pivot_exit(err);
                    *dutyA = BASE_MIN;
                    *dutyB = BASE_MIN;
                    exited = 1;
                }
            } else {
                mid_ok = 0;
            }
            if (!exited) pivot_hold--;
        } else if (pivot_need_mid) {
            /* 锐角盲转窗用完还没对准：必须"外侧全灭 + 中间见线"连续 2 拍才退。
             * 不能用"中间任一路亮"——进弯时原主线本来就压着 X4/X5，
             * 那条判据会让车原地没转就退出，方向没变，PD 只能沿原线走。 */
            if (pivot_cnt >= PIVOT_MIN_TURN && mid_zone) {
                if (++mid_ok >= 2) {
                    pivot_exit(err);
                    *dutyA = BASE_MIN;
                    *dutyB = BASE_MIN;
                    exited = 1;
                }
            } else {
                mid_ok = 0;
            }
        } else if (pivot_cnt >= PIVOT_MIN_TURN && mid_zone) {
            /* 丢线进入：线回到中间区才退。旧判据"任意探头见线就退"会在
             * 车转过 135°、身后旧线扫进外侧探头时退出 → 顺着原路倒着跑。 */
            if (++mid_ok >= 2) {
                pivot_exit(err);
                *dutyA = BASE_MIN;
                *dutyB = BASE_MIN;
                exited = 1;
            }
        } else if (pivot_back && line != 0x00) {
            /* 已经反向回扫还没对准线：不再苛求居中，任意见线即退，
             * 保证扫掠总能闭环，不会原地无限自转 */
            mid_ok = 0;
            pivot_exit(err);
            *dutyA = BASE_MIN;
            *dutyB = BASE_MIN;
            exited = 1;
        } else {
            mid_ok = 0;
        }

        if (!exited && pivot_acute && pivot_cnt >= PIVOT_ACUTE_TURN) {
            /* 锐角转够硬上限还没对到新线：直接交回 PD，让 PD 自己收尾。
             * 绝不换向回扫——原主线一直在视野里，反向必扫回原线，
             * 退出后车头对着来路，就成了"倒着跑"。 */
            pivot_exit(err);
            *dutyA = BASE_MIN;
            *dutyB = BASE_MIN;
            exited = 1;
        }

        if (!exited) {
            /* 去程 1.6s 找不到 → 反向回扫 0.6s → 再朝原方向 1.6s……
             * 每轮净转 1s，一圈圈扫过去总能扫到线，且不会停下来。 */
            uint16_t limit = pivot_back ? PIVOT_RETRY : PIVOT_TIMEOUT;
            /* 对准判据太苛刻时（线粗、斜着扫进来压 3 路），400ms 还没对准就
             * 放宽成"任意中间路见线"，别原地一直自转 */
            if (pivot_need_mid && pivot_cnt >= PIVOT_ACUTE_MAX) {
                pivot_need_mid = 0;
                mid_ok = 0;
            }
            if (++pivot_cnt >= limit) {
                pivot_cnt  = 0;
                pivot_back = pivot_back ? 0 : 1;
                pivot_dir  = (int8_t)(-pivot_dir);
                /* 换向说明原方向扫了 1.6s 都没把线带回中间，
                 * 放弃"必须居中"的苛刻退出，改回任意见线即退兜底 */
                pivot_need_mid = 0;
                pid_reset();
            }
            if (pivot_dir < 0) {          /* dir<0：线在 X1/左侧 → 车头朝左拧 */
                *dutyA = (int16_t)( PIVOT_SIGN * PIVOT_SPEED);
                *dutyB = (int16_t)(-PIVOT_SIGN * PIVOT_SPEED);
            } else {                      /* dir>0：线在 X8/右侧 → 车头朝右拧 */
                *dutyA = (int16_t)(-PIVOT_SIGN * PIVOT_SPEED);
                *dutyB = (int16_t)( PIVOT_SIGN * PIVOT_SPEED);
            }
        }
    }

    cur_dutyA = *dutyA;
    cur_dutyB = *dutyB;
}
