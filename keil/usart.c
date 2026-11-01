#include "usart.h"

#define RX_BUF_SIZE 128
#define TX_BUF_SIZE 256

static volatile uint8_t  rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;   /* 中断写入 */
static volatile uint16_t rx_tail = 0;   /* 主循环读出 */

static volatile uint8_t  tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_head = 0;   /* 主循环入队 */
static volatile uint16_t tx_tail = 0;   /* TX 中断搬运 */

void USART_Init(void)
{

    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

/* 非阻塞发送：只入队，实际搬运交给 TX 中断。
 * 原先是阻塞等 BUSY 清零，一帧 60 字节约 5.2ms；
 * 遥测提到 100fps（10ms 一帧）后会把控制周期拖成 15ms，
 * PID 里的 dt 就与真实周期不符了。
 * 生产者/消费者各自只写自己的下标，Cortex-M0+ 16 位访问是原子的，无需关中断。 */
void USART_SendChar(char c)
{
    uint16_t next = (uint16_t)((tx_head + 1) % TX_BUF_SIZE);

    if (next == tx_tail) {
        return;                     /* 缓冲满则丢，不覆盖 */
    }
    tx_buf[tx_head] = (uint8_t)c;
    tx_head = next;

    DL_UART_Main_enableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_TX);
}

void USART_SendString(const char *s)
{
    while (*s) {
        USART_SendChar(*s++);
    }
}

int16_t USART_GetChar(void)
{
    if (rx_head == rx_tail) {
        return -1;
    }
    uint8_t c = rx_buf[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1) % RX_BUF_SIZE);
    return (int16_t)c;
}

/* UART1 中断：RX 只入队，TX 从环形缓冲搬一个字节，都快进快出。
 *
 * IIDX 优先级：FEFG(0x02)/OEFG(0x05) > RXIFG(0x0B) > TXIFG(0x0C)。
 * 帧错误/溢出这类 error 的优先级比 RX 还高，一旦发生又不清标志，
 * 每次进中断都会返回 error 分支，RX 就再也读不到 —— 命令全丢。
 * 所以这里先无条件把接收侧抽干，再按 IIDX 处理 TX 与错误标志。 */
void UART_1_INST_IRQHandler(void)
{
    if (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {
        while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {
            uint8_t c = DL_UART_Main_receiveData(UART_1_INST);
            uint16_t next = (uint16_t)((rx_head + 1) % RX_BUF_SIZE);
            if (next != rx_tail) {      /* 缓冲满则丢字节，不覆盖 */
                rx_buf[rx_head] = c;
                rx_head = next;
            }
        }
        return;                         /* TX 留到下一拍，优先保证下行命令不丢 */
    }

    switch (DL_UART_Main_getPendingInterrupt(UART_1_INST)) {
        case DL_UART_MAIN_IIDX_TX:
            if (tx_tail != tx_head) {
                DL_UART_Main_transmitData(UART_1_INST, tx_buf[tx_tail]);
                tx_tail = (uint16_t)((tx_tail + 1) % TX_BUF_SIZE);
            }
            if (tx_tail == tx_head) {       /* 发空了就关 TX 中断，避免空转 */
                DL_UART_Main_disableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_TX);
            }
            break;
        case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
        case DL_UART_MAIN_IIDX_FRAMING_ERROR:
        case DL_UART_MAIN_IIDX_PARITY_ERROR:
        case DL_UART_MAIN_IIDX_NOISE_ERROR:
        case DL_UART_MAIN_IIDX_BREAK_ERROR:
            /* 链路噪声/溢出：读一次数据寄存器清标志，否则中断会反复触发卡死主循环 */
            (void)DL_UART_Main_receiveData(UART_1_INST);
            break;
        default:
            break;
    }
}
