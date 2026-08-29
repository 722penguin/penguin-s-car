#include "encoder.h"
#include "stm32f10x_gpio.h"

/*
 * 初始化TIM2编码器接口
 * 使用PA0和PA1
 */
void Encoder_Init_TIM2(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 开启TIM2和GPIOA时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* PA0、PA1作为编码器输入 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 配置TIM2计数器 */
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* 将TIM2设置为编码器模式 */
    TIM_EncoderInterfaceConfig(
        TIM2,
        TIM_EncoderMode_TI12,
        TIM_ICPolarity_Rising,
        TIM_ICPolarity_Rising
    );

    /* 设置输入滤波，减少干扰 */
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    /* 计数值从0开始 */
    TIM_SetCounter(TIM2, 0);

    /* 启动TIM2 */
    TIM_Cmd(TIM2, ENABLE);
}

/*
 * 初始化TIM3编码器接口
 * 使用PA6和PA7
 */
void Encoder_Init_TIM3(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 开启TIM3和GPIOA时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* PA6、PA7作为编码器输入 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 配置TIM3计数器 */
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* 将TIM3设置为编码器模式 */
    TIM_EncoderInterfaceConfig(
        TIM3,
        TIM_EncoderMode_TI12,
        TIM_ICPolarity_Rising,
        TIM_ICPolarity_Rising
    );

    /* 设置输入滤波，减少干扰 */
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    /* 计数值从0开始 */
    TIM_SetCounter(TIM3, 0);

    /* 启动TIM3 */
    TIM_Cmd(TIM3, ENABLE);
}

/*
 * 读取编码器在本次测量时间内产生的计数
 *
 * TIMX = 2：读取TIM2
 * TIMX = 3：读取TIM3
 */
int Read_Encoder(u8 TIMX)
{
    int encoder_value;

    switch (TIMX)
    {
        case 2:
            /*
             * 读取TIM2当前计数。
             * short是有符号数，因此可以表示正转和反转。
             */
            encoder_value = (short)TIM_GetCounter(TIM2);

            /* 读取以后清零，开始下一轮计数 */
            TIM_SetCounter(TIM2, 0);
            break;

        case 3:
            encoder_value = (short)TIM_GetCounter(TIM3);
            TIM_SetCounter(TIM3, 0);
            break;

        default:
            encoder_value = 0;
            break;
    }

    return encoder_value;
}
