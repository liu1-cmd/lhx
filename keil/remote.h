#ifndef REMOTE_H
#define REMOTE_H

#include <stdint.h>

/* 蓝牙遥控（与调参共用同一路 UART、同一条命令行通道）
 *
 * 下行命令（手机 -> 车），一行一条：
 *   f  前进        b  后退
 *   l  左转        r  右转        x  停车
 *
 *   f/b/l/r 是"按住型"指令：连续发就一直走；一旦命令断流超过 800ms，
 *   自动停车（防止手机断开后小车一直冲出去）。
 *
 *   mode=1  进入遥控模式（默认 0 = 自动循迹），也接受 mode 1 / mode:1 / m1
 *   mode=0  回到自动循迹（会软启动，不会突然满速），也接受 m0
 *   spd=1200  遥控速度（300~3200，独立于循迹的 base，互不影响）
 *
 * 遥控是否真的生效，看遥测 ch6：2 = 遥控模式，0/1 = 自动循迹。
 */

#define REMOTE_AUTO    0
#define REMOTE_MANUAL  1

void    Remote_Init(void);
uint8_t Remote_Mode(void);

/* 解析一条命令行：返回 1=本模块已处理，0=不是遥控命令（交给调参模块） */
uint8_t Remote_HandleLine(const char *s);

/* 每 5ms 调一次，仅手动模式下使用：算两轮带符号占空比 */
void Remote_Update(int16_t *dutyA, int16_t *dutyB);

#endif
