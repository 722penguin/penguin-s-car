#ifndef __CONTROL_SYSTEMS_H
#define __CONTROL_SYSTEMS_H

#include "stm32f10x.h"

/* One speed unit means one encoder count per 100 ms. */
#define CONTROL_PERIOD_MS       100U
#define CONTROL_DEFAULT_SPEED   100
#define CONTROL_PWM_MAX         7199
#define CONTROL_TIMEOUT_MS      1000U

extern int Target_Left;
extern int Target_Right;
extern int Left_Speed;
extern int Right_Speed;
extern int Left_PWM;
extern int Right_PWM;

/* Called once per millisecond from SysTick_Handler. */
void Control_Tick_1ms(void);

/* Called repeatedly from main(). */
void Control_Task(void);

/* Executes one encoder -> PI -> PWM control cycle. */
void System_Control(void);

/* Incremental PI controllers. */
int Incremental_PI_Left(int actual, int target);
int Incremental_PI_Right(int actual, int target);
void Control_Reset_PI(void);

/* Chassis motion interface for later sensors/Hi3861 code. */
void Control_Set_Target(int left, int right);
void Control_Stop(void);
void Control_Forward(int speed);
void Control_Backward(int speed);
void Control_Turn_Left(int speed);
void Control_Turn_Right(int speed);
void Control_Spin_Left(int speed);
void Control_Spin_Right(int speed);

/* USART1 single-character commands: F/B/L/R/Q/E/S. */
void Control_Process_Command(u8 command);

#endif
