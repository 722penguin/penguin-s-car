#include "motor.h"

#define MOTOR_PWM_MAX  7199
#define CAR_SPEED      4000

/*
 * ???????????
 * PB13?PB14?????????
 */
void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin =
        GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    AIN = 0;
    BIN = 0;
}

/*
 * ???TIM4?PWM
 * PB6??TIM4_CH1
 * PB7??TIM4_CH2
 */
void PWM_Init(u16 arr, u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    Motor_Init();

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* PB6?PB7????????? */
    GPIO_InitStructure.GPIO_Pin =
        GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* TIM4???? */
    TIM_TimeBaseStructure.TIM_Period = arr;
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    /* PWM???? */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM4, ENABLE);

    TIM_Cmd(TIM4, ENABLE);

    Set_Pwm(0, 0);
}

/* ???? */
static u32 Motor_Abs(int value)
{
    if (value < 0)
    {
        return (u32)(-value);
    }

    return (u32)value;
}

/*
 * ????????
 *
 * ???????????
 * ???????
 * ????:-7199~7199
 */
void Set_Pwm(int motor1, int motor2)
{
    u32 pwm1;
    u32 pwm2;

    /* ????????? */
    if (motor1 > MOTOR_PWM_MAX)
    {
        motor1 = MOTOR_PWM_MAX;
    }
    else if (motor1 < -MOTOR_PWM_MAX)
    {
        motor1 = -MOTOR_PWM_MAX;
    }

    if (motor2 > MOTOR_PWM_MAX)
    {
        motor2 = MOTOR_PWM_MAX;
    }
    else if (motor2 < -MOTOR_PWM_MAX)
    {
        motor2 = -MOTOR_PWM_MAX;
    }

    pwm1 = Motor_Abs(motor1);
    pwm2 = Motor_Abs(motor2);

    /* ????? */
    if (motor2 >= 0)
    {
        AIN = 0;
        PWMA = pwm2;
    }
    else
    {
        AIN = 1;
        PWMA = MOTOR_PWM_MAX - pwm2;
    }

    /* ????? */
    if (motor1 >= 0)
    {
        BIN = 0;
        PWMB = pwm1;
    }
    else
    {
        BIN = 1;
        PWMB = MOTOR_PWM_MAX - pwm1;
    }
}

void Car_Stop(void)
{
    Set_Pwm(0, 0);
}

void Car_Forward(void)
{
    Set_Pwm(CAR_SPEED, CAR_SPEED);
}

void Car_Backward(void)
{
    Set_Pwm(-CAR_SPEED, -CAR_SPEED);
}

/* ????:?????? */
void Car_Left(void)
{
    Set_Pwm(0, CAR_SPEED);
}

void Car_Right(void)
{
    Set_Pwm(CAR_SPEED, 0);
}
