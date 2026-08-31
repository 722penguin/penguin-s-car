#ifndef __USART_H
#define __USART_H
#include "stdio.h"
#include "sys.h"

#define USART_REC_LEN 200
#define EN_USART1_RX 1
#define USART1_RX_FIFO_SIZE 128U
#define USART1_TX_FIFO_SIZE 256U

#define USART1_FRAME_HEAD1 0xA5U
#define USART1_FRAME_HEAD2 0x5AU
#define USART1_FRAME_SPEED 0x01U

extern u8 USART_RX_BUF[USART_REC_LEN];
extern u8 USART_RX_STA;

void uart_init(u32 bound);
u8 USART1_ReadByte(u8 *byte);
u8 USART1_TakeRxOverflow(void);
u8 USART1_TakeError(void);
u8 USART1_Send(const u8 *data, u16 length);
u8 USART1_SendFrame(u8 type, const u8 *payload, u8 length);

#endif
