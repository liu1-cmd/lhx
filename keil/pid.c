#include "pid.h"

void PID_Init(PID *p, float kp, float ki, float kd,
              float out_min, float out_max, float int_max)//pd
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->out_min = out_min;
    p->out_max = out_max;
    p->int_max = int_max;
    p->integral = 0.0f;
    p->last_err = 0.0f;
    p->last_p = p->last_i = p->last_d = 0.0f;
}

/* error: 当前误差; dt: 控制周期(秒)，必须与实际循环周期一致 */
float PID_Compute(PID *p, float error, float dt)
{
    float derivative = error - p->last_err;
    p->last_err = error;

    /* 积分累加 + 抗饱和限幅 */
    p->integral += error * dt;
    if (p->integral > p->int_max)  p->integral = p->int_max;
    if (p->integral < -p->int_max) p->integral = -p->int_max;

    p->last_p = p->kp * error;
    p->last_i = p->ki * p->integral;
    p->last_d = p->kd * derivative / dt;

    float out = p->last_p + p->last_i + p->last_d;

    /* 输出限幅 */
    if (out > p->out_max) out = p->out_max;
    if (out < p->out_min) out = p->out_min;
    return out;
}
