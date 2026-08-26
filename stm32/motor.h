#ifndef __MOTOR_H
#define __MOTOR_H

#include "sys.h"

/*
 * 根据小车原理图：
 * PB13、PB14控制电机方向
 * PB6、PB7输出TIM4 PWM
 */
#define AIN   PBout(13)
#define BIN   PBout(14)

#define PWMA  TIM4->CCR1
#define PWMB  TIM4->CCR2

void Motor_Init(void);
void PWM_Init(u16 arr, u16 psc);
void Set_Pwm(int motor1, int motor2);

void Car_Stop(void);
void Car_Forward(void);
void Car_Backward(void);
void Car_Left(void);
void Car_Right(void);

#endif
