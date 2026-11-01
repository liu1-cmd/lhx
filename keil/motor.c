#include "motor.h"

#define MOTOR_DUTY_MAX 3200   /* PWM 周期，与 SysConfig 一致 */

/* 带符号占空比接口：正=前进，负=反转。
 * 原地转（pivot）靠它让两轮反向。
 */
void Motor_Set(int16_t dutyA, int16_t dutyB)
{
    /* 两轮都给 0 = 要求停车：走短接刹车，不让车轮靠惯性继续溜。
     * 切到蓝牙遥控、遥控信号断流、循迹收尾都会走到这里。 */
    if (dutyA == 0 && dutyB == 0) {
        DL_TimerA_setCaptureCompareValue(PWM_0_INST, 0, DL_TIMER_CC_0_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_0_INST, 0, DL_TIMER_CC_1_INDEX);
        Motor_Stop();
        return;
    }

    if (dutyA >= 0) {                    /* A 轮正转 */
        DL_GPIO_setPins(AIN_PORT, AIN_AIN1_PIN);
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN2_PIN);
    } else {                             /* A 轮反转 */
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN);
        DL_GPIO_setPins(AIN_PORT, AIN_AIN2_PIN);
        dutyA = -dutyA;
    }

    if (dutyB >= 0) {                    /* B 轮正转 */
        DL_GPIO_setPins(BIN_PORT, BIN_BIN1_PIN);
        DL_GPIO_clearPins(BIN_PORT, BIN_BIN2_PIN);
    } else {                             /* B 轮反转 */
        DL_GPIO_clearPins(BIN_PORT, BIN_BIN1_PIN);
        DL_GPIO_setPins(BIN_PORT, BIN_BIN2_PIN);
        dutyB = -dutyB;
    }

    if (dutyA > MOTOR_DUTY_MAX) dutyA = MOTOR_DUTY_MAX;
    if (dutyB > MOTOR_DUTY_MAX) dutyB = MOTOR_DUTY_MAX;

    DL_TimerA_setCaptureCompareValue(PWM_0_INST, dutyA, DL_TIMER_CC_0_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, dutyB, DL_TIMER_CC_1_INDEX);
}

void Motor_Forward(uint16_t dutyA, uint16_t dutyB)
{
    Motor_Set((int16_t)dutyA, (int16_t)dutyB);
}

void Motor_Backward(uint16_t dutyA, uint16_t dutyB)
{
    Motor_Set(-(int16_t)dutyA, -(int16_t)dutyB);
}

void Motor_Stop(void)
{
    /* 四输入全高 = 刹车（短接） */
    DL_GPIO_setPins(AIN_PORT, AIN_AIN1_PIN);
    DL_GPIO_setPins(AIN_PORT, AIN_AIN2_PIN);
    DL_GPIO_setPins(BIN_PORT, BIN_BIN1_PIN);
    DL_GPIO_setPins(BIN_PORT, BIN_BIN2_PIN);
}
