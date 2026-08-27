#ifndef __ENCODER_H
#define __ENCODER_H

#include "sys.h"

/* 16位定时器最大计数值 */
#define ENCODER_TIM_PERIOD 65535

/* 初始化左右轮编码器 */
void Encoder_Init_TIM2(void);
void Encoder_Init_TIM3(void);

/* 读取指定定时器的编码器计数 */
int Read_Encoder(u8 TIMX);

#endif
