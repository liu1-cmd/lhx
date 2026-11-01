#ifndef USART_H
#define USART_H

#include "ti_msp_dl_config.h"

/* 底层串口传输层（UART1: PB6=TX, PB7=RX, 115200）
 * 只负责收发字节，不关心协议。
 */

void USART_Init(void);            /* 使能 RX 中断 + NVIC（SysConfig 没生成，这里补） */
void USART_SendChar(char c);      /* 非阻塞：写入发送环形缓冲，由 TX 中断搬运 */
void USART_SendString(const char *s);

/* 从接收环形缓冲取一个字节；无数据返回 -1 */
int16_t USART_GetChar(void);

#endif
