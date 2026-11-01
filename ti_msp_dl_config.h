/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
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

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMA0
#define PWM_0_INST_IRQHandler                                   TIMA0_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMA0_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                             32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOA
#define GPIO_PWM_0_C0_PIN                                          DL_GPIO_PIN_8
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM19)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM19_PF_TIMA0_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOA
#define GPIO_PWM_0_C1_PIN                                          DL_GPIO_PIN_9
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM20)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM20_PF_TIMA0_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for UART_1 */
#define UART_1_INST                                                        UART1
#define UART_1_INST_FREQUENCY                                           32000000
#define UART_1_INST_IRQHandler                                  UART1_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOB
#define GPIO_UART_1_TX_PORT                                                GPIOB
#define GPIO_UART_1_RX_PIN                                         DL_GPIO_PIN_7
#define GPIO_UART_1_TX_PIN                                         DL_GPIO_PIN_6
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM24)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM23)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM24_PF_UART1_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM23_PF_UART1_TX
#define UART_1_BAUD_RATE                                                (115200)
#define UART_1_IBRD_32_MHZ_115200_BAUD                                      (17)
#define UART_1_FBRD_32_MHZ_115200_BAUD                                      (23)





/* Port definition for Pin Group LEDB */
#define LEDB_PORT                                                        (GPIOB)

/* Defines for PIN_22: GPIOB.22 with pinCMx 50 on package pin 21 */
#define LEDB_PIN_22_PIN                                         (DL_GPIO_PIN_22)
#define LEDB_PIN_22_IOMUX                                        (IOMUX_PINCM50)
/* Port definition for Pin Group OLED */
#define OLED_PORT                                                        (GPIOB)

/* Defines for OLED_SCL: GPIOB.11 with pinCMx 28 on package pin 63 */
#define OLED_OLED_SCL_PIN                                       (DL_GPIO_PIN_11)
#define OLED_OLED_SCL_IOMUX                                      (IOMUX_PINCM28)
/* Defines for OLED_SDA: GPIOB.10 with pinCMx 27 on package pin 62 */
#define OLED_OLED_SDA_PIN                                       (DL_GPIO_PIN_10)
#define OLED_OLED_SDA_IOMUX                                      (IOMUX_PINCM27)
/* Port definition for Pin Group AIN */
#define AIN_PORT                                                         (GPIOA)

/* Defines for AIN1: GPIOA.0 with pinCMx 1 on package pin 33 */
#define AIN_AIN1_PIN                                             (DL_GPIO_PIN_0)
#define AIN_AIN1_IOMUX                                            (IOMUX_PINCM1)
/* Defines for AIN2: GPIOA.1 with pinCMx 2 on package pin 34 */
#define AIN_AIN2_PIN                                             (DL_GPIO_PIN_1)
#define AIN_AIN2_IOMUX                                            (IOMUX_PINCM2)
/* Port definition for Pin Group BIN */
#define BIN_PORT                                                         (GPIOB)

/* Defines for BIN1: GPIOB.0 with pinCMx 12 on package pin 47 */
#define BIN_BIN1_PIN                                             (DL_GPIO_PIN_0)
#define BIN_BIN1_IOMUX                                           (IOMUX_PINCM12)
/* Defines for BIN2: GPIOB.1 with pinCMx 13 on package pin 48 */
#define BIN_BIN2_PIN                                             (DL_GPIO_PIN_1)
#define BIN_BIN2_IOMUX                                           (IOMUX_PINCM13)
/* Defines for X1: GPIOA.13 with pinCMx 35 on package pin 6 */
#define linefollower_X1_PORT                                             (GPIOA)
#define linefollower_X1_PIN                                     (DL_GPIO_PIN_13)
#define linefollower_X1_IOMUX                                    (IOMUX_PINCM35)
/* Defines for X2: GPIOA.14 with pinCMx 36 on package pin 7 */
#define linefollower_X2_PORT                                             (GPIOA)
#define linefollower_X2_PIN                                     (DL_GPIO_PIN_14)
#define linefollower_X2_IOMUX                                    (IOMUX_PINCM36)
/* Defines for X3: GPIOA.15 with pinCMx 37 on package pin 8 */
#define linefollower_X3_PORT                                             (GPIOA)
#define linefollower_X3_PIN                                     (DL_GPIO_PIN_15)
#define linefollower_X3_IOMUX                                    (IOMUX_PINCM37)
/* Defines for X4: GPIOA.17 with pinCMx 39 on package pin 10 */
#define linefollower_X4_PORT                                             (GPIOA)
#define linefollower_X4_PIN                                     (DL_GPIO_PIN_17)
#define linefollower_X4_IOMUX                                    (IOMUX_PINCM39)
/* Defines for X5: GPIOA.10 with pinCMx 21 on package pin 56 */
#define linefollower_X5_PORT                                             (GPIOA)
#define linefollower_X5_PIN                                     (DL_GPIO_PIN_10)
#define linefollower_X5_IOMUX                                    (IOMUX_PINCM21)
/* Defines for X6: GPIOA.11 with pinCMx 22 on package pin 57 */
#define linefollower_X6_PORT                                             (GPIOA)
#define linefollower_X6_PIN                                     (DL_GPIO_PIN_11)
#define linefollower_X6_IOMUX                                    (IOMUX_PINCM22)
/* Defines for X7: GPIOB.2 with pinCMx 15 on package pin 50 */
#define linefollower_X7_PORT                                             (GPIOB)
#define linefollower_X7_PIN                                      (DL_GPIO_PIN_2)
#define linefollower_X7_IOMUX                                    (IOMUX_PINCM15)
/* Defines for X8: GPIOB.3 with pinCMx 16 on package pin 51 */
#define linefollower_X8_PORT                                             (GPIOB)
#define linefollower_X8_PIN                                      (DL_GPIO_PIN_3)
#define linefollower_X8_IOMUX                                    (IOMUX_PINCM16)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_UART_1_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
