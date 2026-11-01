#ifndef PID_H
#define PID_H

/* 通用位置式 PID，与具体应用（循迹/电机）无关，可复用 */
typedef struct {
    float kp, ki, kd;
    float integral;          /* 积分累加 */
    float last_err;          /* 上次误差 */
    float out_min, out_max;  /* 输出限幅（防溢出） */
    float int_max;           /* 积分限幅（抗饱和） */
    float last_p, last_i, last_d; /* 最近一次的 P/I/D 分量，供上位机观测 */
} PID;

void  PID_Init(PID *p, float kp, float ki, float kd,
               float out_min, float out_max, float int_max);
float PID_Compute(PID *p, float error, float dt);

#endif
