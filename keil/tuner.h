#ifndef VOFA_H
#define VOFA_H

#include <stdint.h>

/* VOFA+ FireWater 协议层（依赖 usart.c 传输层）
 *
 * 上行帧（车->电脑），每通道一行、逗号分隔、\r\n 结尾：
 *   ch0: 原始线误差(-7~+7)，数字探头天然是台阶
 *   ch1: P 项 x10
 *   ch2: 滤波后线误差 x10（连续值，用来和 ch0 对比）
 *   ch3: D 项 x10
 *   ch4: dutyA(带符号)
 *   ch5: dutyB(带符号)
 *   ch6: 状态(0=循迹 1=原地找线 2=蓝牙遥控模式)
 *   ch7: kp x10   ch8: ki x10   ch9: kd x10
 *   ch10: 原始 8 位亮灯掩码 0~255（bit7=X1最左 ... bit0=X8最右）
 *         理想模式对照：钝角456=0x1C(28) 锐角1245=0xD8(216) 直角12345=0xF8(248)
 *         全黑(十字)=255  全白(丢线)=0
 *
 * 下行命令（手机/电脑->车），一行一条：
 *   kp=180.0   ki=0.5   kd=0.8   （在线改 PID 增益）
 *   base=1800                    （直线基础速度，500~3200）
 *   tau=0.02                     （线误差一阶低通时间常数，秒；0=关闭）
 *   oled=0/1                     （关/开 OLED 刷新，跑圈时关掉更顺）
 *
 * 蓝牙遥控命令（见 remote.h，与本协议共用串口）：
 *   f/b/l/r/x                    （前进/后退/左转/右转/停车，断流 800ms 自动停）
 *   mode=1 / mode=0              （切到蓝牙遥控 / 切回自动循迹）
 *   spd=1200                     （遥控速度，300~3200）
 */

void VOFA_SendFrame(void);  /* 发一帧遥测，约 50ms 一次即可 */
void VOFA_Poll(void);       /* 每控制周期调用：解析下行命令 */

uint8_t Tuner_OLEDEnabled(void);  /* 1=允许刷新 OLED（可用 oled=0 关闭） */
uint8_t Tuner_EchoEnabled(void);  /* 1=回显 ACK/ERR（可用 echo=1 打开） */

#endif
