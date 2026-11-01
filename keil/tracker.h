#ifndef TRACKER_H
#define TRACKER_H

#include <stdint.h>

void Tracker_Init(void);

/* 每 5ms（200Hz）调用一次。输出带符号占空比给 Motor_Set：
 * 正常循迹两轮为正；pivot 时一正一负。
 */
void Tracker_Update(int16_t *dutyA, int16_t *dutyB);

uint8_t Tracker_State(void);  /* 0=RUN 循迹中, 1=PIVOT 原地找线，供显示 */
int8_t  Tracker_Err(void);    /* 当前线误差 -7~+7，供显示 */
uint8_t Tracker_Line(void);   /* 最近一次原始 8 位亮灯掩码(bit7=X1最左...bit0=X8最右) */

/* 从手动遥控切回自动循迹：内部状态归零并软启动，不重置在线调好的增益 */
void Tracker_Resume(void);

/* 蓝牙在线调参：下发 kp/ki/kd */
void Tracker_SetKP(float kp);
void Tracker_SetKI(float ki);
void Tracker_SetKD(float kd);
void Tracker_GetGains(float *kp, float *ki, float *kd);

/* 外侧探头 pivot 触发开关：0=关(默认，弯道交 PD) 1=旧行为 */
void Tracker_SetOuterPivot(uint8_t on);
uint8_t Tracker_OuterPivot(void);

/* 一阶低通滤波时间常数(秒)，0=关闭滤波，范围 0~0.5，默认 0.02 */
void  Tracker_SetTau(float tau);
float Tracker_GetTau(void);
float Tracker_ErrFiltered(void);   /* 滤波后的连续误差，供观测 */

/* 直线基础速度（占空比，500~3200），默认 2200。
 * 车太快导致探头分辨率跟不上时，在线降到 1600~1800 通常立刻变顺。 */
void Tracker_SetBase(float duty);

/* 取最近一轮的内部量，供 VOFA 遥测 */
void Tracker_GetDebug(int16_t *err, float *p, float *i, float *d,
                      int16_t *dutyA, int16_t *dutyB);

#endif
