#include "sys.h"
#include "usart.h"
#include <stdio.h>

#pragma import(__use_no_semihosting)
struct __FILE { int handle; };
FILE __stdout;
void _sys_exit(int x) { (void)x; }

#if EN_USART1_RX
u8 USART_RX_BUF[USART_REC_LEN];
u8 USART_RX_STA = 0;
static volatile u8 usart1_rx_fifo[USART1_RX_FIFO_SIZE];
static volatile u8 usart1_rx_head = 0;
static volatile u8 usart1_rx_tail = 0;
static volatile u8 usart1_rx_overflow = 0;
static volatile u8 usart1_error = 0;
static volatile u8 usart1_tx_fifo[USART1_TX_FIFO_SIZE];
static volatile u16 usart1_tx_head = 0;
static volatile u16 usart1_tx_tail = 0;

void uart_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    (void)bound;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200U;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_ERR, ENABLE);
    USART_ClearFlag(USART1, USART_FLAG_TC);
    USART_Cmd(USART1, ENABLE);
}

void USART1_IRQHandler(void)
{
    u16 status = USART1->SR;
    if ((status & (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)) != 0U)
    {
        (void)USART1->DR;
        usart1_error = 1;
    }
    else if ((status & USART_FLAG_RXNE) != 0U)
    {
        u8 byte = (u8)USART1->DR;
        u8 next_head = (u8)((usart1_rx_head + 1U) % USART1_RX_FIFO_SIZE);
        if (next_head == usart1_rx_tail) usart1_rx_overflow = 1;
        else { usart1_rx_fifo[usart1_rx_head] = byte; usart1_rx_head = next_head; }
    }

    if ((USART1->CR1 & USART_CR1_TXEIE) != 0U && (status & USART_FLAG_TXE) != 0U)
    {
        if (usart1_tx_tail != usart1_tx_head)
        {
            USART1->DR = usart1_tx_fifo[usart1_tx_tail];
            usart1_tx_tail = (u16)((usart1_tx_tail + 1U) % USART1_TX_FIFO_SIZE);
        }
        else USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    }
}

u8 USART1_ReadByte(u8 *byte)
{
    if (usart1_rx_tail == usart1_rx_head) return 0;
    *byte = usart1_rx_fifo[usart1_rx_tail];
    usart1_rx_tail = (u8)((usart1_rx_tail + 1U) % USART1_RX_FIFO_SIZE);
    return 1;
}

u8 USART1_TakeRxOverflow(void)
{
    u8 value = usart1_rx_overflow;
    usart1_rx_overflow = 0;
    return value;
}

u8 USART1_TakeError(void)
{
    u8 value = usart1_error;
    usart1_error = 0;
    return value;
}

u8 USART1_Send(const u8 *data, u16 length)
{
    u16 i;
    u16 next_head;
    u16 used;
    u16 free_space;
    u32 primask = __get_PRIMASK();
    __disable_irq();
    used = (u16)((usart1_tx_head + USART1_TX_FIFO_SIZE - usart1_tx_tail) % USART1_TX_FIFO_SIZE);
    free_space = (u16)(USART1_TX_FIFO_SIZE - 1U - used);
    if (length > free_space)
    {
        if (primask == 0U) __enable_irq();
        return 0;
    }
    for (i = 0; i < length; i++)
    {
        next_head = (u16)((usart1_tx_head + 1U) % USART1_TX_FIFO_SIZE);
        if (next_head == usart1_tx_tail)
        {
            if (primask == 0U) __enable_irq();
            return 0;
        }
        usart1_tx_fifo[usart1_tx_head] = data[i];
        usart1_tx_head = next_head;
    }
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    if (primask == 0U) __enable_irq();
    return 1;
}

u8 USART1_SendFrame(u8 type, const u8 *payload, u8 length)
{
    u8 frame[USART1_TX_FIFO_SIZE];
    u8 i;
    u8 checksum = (u8)(length + type);
    if ((u16)length + 5U > USART1_TX_FIFO_SIZE) return 0;
    frame[0] = USART1_FRAME_HEAD1;
    frame[1] = USART1_FRAME_HEAD2;
    frame[2] = length;
    frame[3] = type;
    for (i = 0; i < length; i++) { frame[4U + i] = payload[i]; checksum = (u8)(checksum + payload[i]); }
    frame[4U + length] = checksum;
    return USART1_Send(frame, (u16)length + 5U);
}

/* Route printf output to USART1 so serial diagnostics are visible on the PC. */
int fputc(int ch, FILE *f)
{
    (void)f;
    while ((USART1->SR & USART_FLAG_TC) == 0U) {}
    USART1->DR = (u8)ch;
    return ch;
}
#endif
