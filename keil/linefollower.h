#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include "ti_msp_dl_config.h"

uint8_t LF_Read(void);     // 读 X1(最左)~X8(最右)，1=看见黑线
int8_t LF_GetError(void);  // 线位置误差：负=偏左，正=偏右，-7~+7
int8_t LF_ErrorFrom(uint8_t x); // 用已经读到的 x 算误差（避免二次采样）

#endif
