#include "linefollower.h"

uint8_t LF_Read(void)
{
    uint8_t x = 0;
    x |= (DL_GPIO_readPins(linefollower_X8_PORT, linefollower_X8_PIN)) ?  0: 0x01;
    x |= (DL_GPIO_readPins(linefollower_X7_PORT, linefollower_X7_PIN)) ?  0: 0x02;
    x |= (DL_GPIO_readPins(linefollower_X6_PORT, linefollower_X6_PIN)) ?  0: 0x04;
    x |= (DL_GPIO_readPins(linefollower_X5_PORT, linefollower_X5_PIN)) ?  0: 0x08;
    x |= (DL_GPIO_readPins(linefollower_X4_PORT, linefollower_X4_PIN)) ?  0: 0x10;
    x |= (DL_GPIO_readPins(linefollower_X3_PORT, linefollower_X3_PIN)) ?  0: 0x20;
    x |= (DL_GPIO_readPins(linefollower_X2_PORT, linefollower_X2_PIN)) ?  0: 0x40;
    x |= (DL_GPIO_readPins(linefollower_X1_PORT, linefollower_X1_PIN)) ?  0: 0x80;
    return x;
}

/* 线位置误差：负=线偏左，正=线偏右，范围 -7~+7。
 * 权重按 X1(最左)=-7 ... X8(最右)=+7，多路压线时取平均。
 * 全白（丢线）：保持上次误差，继续朝最后看到的方向找；
 * 全黑（十字/起点）：误差视为居中返回 0，但不清 last，
 *   这样十字过后若丢线，仍能记住线最后偏向哪一侧。
 */
int8_t LF_ErrorFrom(uint8_t x)
{
    static const int8_t w[8] = {-7, -5, -3, -1, 1, 3, 5, 7}; /* X1..X8 */
    static int8_t last = 0;

    if (x == 0x00) return last;             /* 丢线：保持上次误差 */
    if (x == 0xFF) return 0;                /* 全黑：十字居中，保留 last */

    int16_t sum = 0;
    uint8_t n = 0;
    for (uint8_t i = 0; i < 8; i++) {			//判断误差
        if (x & (0x80 >> i)) { sum += w[i]; n++; }
    }
    last = (int8_t)(sum / n);
    return last;
}

int8_t LF_GetError(void)
{
    return LF_ErrorFrom(LF_Read());
}
