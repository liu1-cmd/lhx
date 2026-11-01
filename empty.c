/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "OLED.h"
#include "motor.h"
#include "linefollower.h"
#include "tracker.h"
#include "usart.h"
#include "tuner.h"
#include "remote.h"
#include "delay.h"

/* 主循环只做三件事：初始化、按周期调度、慢速显示。
 * 循迹策略在 tracker.c，传感器在 linefollower.c，
 * 电机在 motor.c，PID 在 pid.c。
 *
 * 控制（PID+电机输出）放在 SysTick 5ms 中断里跑：
 * dt 由硬件定时器保证严格等于 CTRL_DT，OLED 刷屏、串口轮询、
 * delay_ms 抖动都不再影响控制周期，D 项不会因 dt 波动产生尖刺。
 * UART 中断默认优先级高于 SysTick（CMSIS 把 SysTick 设为最低），
 * 遥测发送只会打断控制几微秒，不会拖慢控制。
 */
volatile uint32_t g_ctrl_ticks;   /* 控制周期计数，5ms +1，主循环用它定拍 */

void SysTick_Handler(void)
{
    static int16_t dutyA, dutyB;
    if (Remote_Mode() == REMOTE_MANUAL) {
        Remote_Update(&dutyA, &dutyB);   /* 蓝牙手动遥控 */
    } else {
        Tracker_Update(&dutyA, &dutyB);  /* 策略计算 */
    }
    Motor_Set(dutyA, dutyB);             /* 执行 */
    g_ctrl_ticks++;
}

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "line:");
    USART_Init();                        
    USART_SendString("BOOT\r\n");
    Tracker_Init();
    Remote_Init();                       /* 默认自动循迹模式 */

    g_ctrl_ticks = 0;
    SysTick_Config(CPUCLK_FREQ / 200);   /* 5ms 控制周期（200Hz） */

    while (1) {
        static uint32_t lastFrame = 0, lastOled = 0;
        VOFA_Poll();                      /* 处理蓝牙下发的调参/控制命令 */

        /* 遥测 100fps（每 2 个控制周期一帧），不占控制中断时间 */
        if (g_ctrl_ticks - lastFrame >= 2u) {
            lastFrame = g_ctrl_ticks;
            VOFA_SendFrame();
        }

        /* OLED 软件 I2C 很慢，放主循环里 0.5s 刷一次，
         * 它再阻塞也只拖累显示，不再打断控制周期。 */
        if (g_ctrl_ticks - lastOled >= 100u && Tuner_OLEDEnabled()) {
            lastOled = g_ctrl_ticks;
            OLED_ShowHexNum(2, 1, LF_Read(), 2);        /* 8 路传感器（十六进制） */
            OLED_ShowSignedNum(3, 1, Tracker_Err(), 2); /* 线误差 */
            OLED_ShowNum(4, 1, Tracker_State(), 1);     /* 0=循迹 1=原地转 */
            OLED_ShowNum(5, 1, Remote_Mode(), 1);       /* 0=自动 1=蓝牙遥控 */
        }

        delay_ms(1);                      /* 让出 CPU，轮询频率 1ms 足够 */
    }
}
