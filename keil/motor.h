#ifndef MOTOR_H
#define MOTOR_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* 带符号占空比：正=前进，负=反转，范围 -3200~+3200 */
void Motor_Set(int16_t dutyA, int16_t dutyB);

void Motor_Forward(uint16_t dutyA, uint16_t dutyB);   // 前进：A轮 + B轮
void Motor_Backward(uint16_t dutyA, uint16_t dutyB);  // 后退
void Motor_Stop(void);                               // 刹车

#endif
