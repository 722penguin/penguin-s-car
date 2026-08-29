#include "control_systems.h"
#include "encoder.h"
#include "motor.h"
#include "usart.h"
#include <stdio.h>

/* Target and measured speeds: encoder counts per 100 ms. */
int Target_Left = 0;
int Target_Right = 0;
int Left_Speed = 0;
int Right_Speed = 0;

/* PI controller output, limited to -7199 .. 7199. */
int Left_PWM = 0;
int Right_PWM = 0;

/* Values that must survive between incremental PI calculations. */
static int left_pi_pwm = 0;
static int right_pi_pwm = 0;
static int left_last_error = 0;
static int right_last_error = 0;

/* Variables shared between SysTick interrupt and the main loop. */
static volatile u16 control_millis = 0;
static volatile u16 command_millis = 0;
static volatile u8 control_ready = 0;
static volatile u8 command_active = 0;
static volatile u8 timeout_pending = 0;

static int Limit_Speed(int speed)
{
    /* Conservative software limit before the real car is calibrated. */
    if (speed > 300)
    {
        return 300;
    }
    if (speed < -300)
    {
        return -300;
    }
    return speed;
}

static int Limit_PWM(int pwm)
{
    if (pwm > CONTROL_PWM_MAX)
    {
        return CONTROL_PWM_MAX;
    }
    if (pwm < -CONTROL_PWM_MAX)
    {
        return -CONTROL_PWM_MAX;
    }
    return pwm;
}

void Control_Reset_PI(void)
{
    left_pi_pwm = 0;
    right_pi_pwm = 0;
    left_last_error = 0;
    right_last_error = 0;
    Left_PWM = 0;
    Right_PWM = 0;
}

int Incremental_PI_Left(int actual, int target)
{
    const float Kp = 7.0f;
    const float Ki = 0.016f;
    int error = target - actual;

    left_pi_pwm += (int)(Kp * (error - left_last_error) + Ki * error);
    left_last_error = error;
    left_pi_pwm = Limit_PWM(left_pi_pwm);

    return left_pi_pwm;
}

int Incremental_PI_Right(int actual, int target)
{
    const float Kp = 7.0f;
    const float Ki = 0.016f;
    int error = target - actual;

    right_pi_pwm += (int)(Kp * (error - right_last_error) + Ki * error);
    right_last_error = error;
    right_pi_pwm = Limit_PWM(right_pi_pwm);

    return right_pi_pwm;
}

void Control_Set_Target(int left, int right)
{
    int new_left = Limit_Speed(left);
    int new_right = Limit_Speed(right);

    /* Do not carry the previous motion's PI output into a new direction. */
    if (new_left != Target_Left || new_right != Target_Right)
    {
        Control_Reset_PI();
    }

    Target_Left = new_left;
    Target_Right = new_right;

    command_millis = 0;
    timeout_pending = 0;
    command_active = 1;
}

void Control_Stop(void)
{
    Target_Left = 0;
    Target_Right = 0;
    command_active = 0;
    command_millis = 0;
    timeout_pending = 0;

    Control_Reset_PI();
    Set_Pwm(0, 0);
}

void Control_Forward(int speed)
{
    if (speed < 0) speed = -speed;
    Control_Set_Target(speed, speed);
}

void Control_Backward(int speed)
{
    if (speed < 0) speed = -speed;
    Control_Set_Target(-speed, -speed);
}

void Control_Turn_Left(int speed)
{
    if (speed < 0) speed = -speed;
    Control_Set_Target(0, speed);
}

void Control_Turn_Right(int speed)
{
    if (speed < 0) speed = -speed;
    Control_Set_Target(speed, 0);
}

void Control_Spin_Left(int speed)
{
    if (speed < 0) speed = -speed;
    Control_Set_Target(-speed, speed);
}

void Control_Spin_Right(int speed)
{
    if (speed < 0) speed = -speed;
    Control_Set_Target(speed, -speed);
}

void Control_Process_Command(u8 command)
{
    /* Accept lower-case commands too. */
    if (command >= 'a' && command <= 'z')
    {
        command = (u8)(command - 'a' + 'A');
    }

    switch (command)
    {
        case 'F':
            Control_Forward(CONTROL_DEFAULT_SPEED);
            printf("ACK forward\r\n");
            break;

        case 'B':
            Control_Backward(CONTROL_DEFAULT_SPEED);
            printf("ACK backward\r\n");
            break;

        case 'L':
            Control_Turn_Left(CONTROL_DEFAULT_SPEED);
            printf("ACK left\r\n");
            break;

        case 'R':
            Control_Turn_Right(CONTROL_DEFAULT_SPEED);
            printf("ACK right\r\n");
            break;

        case 'Q':
            Control_Spin_Left(CONTROL_DEFAULT_SPEED);
            printf("ACK spin-left\r\n");
            break;

        case 'E':
            Control_Spin_Right(CONTROL_DEFAULT_SPEED);
            printf("ACK spin-right\r\n");
            break;

        case 'S':
            Control_Stop();
            printf("ACK stop\r\n");
            break;

        case '\r':
        case '\n':
            break;

        default:
            Control_Stop();
            printf("Unknown command; stopped\r\n");
            break;
    }
}

void System_Control(void)
{
    Left_Speed = Read_Encoder(2);
    Right_Speed = Read_Encoder(3);

    if (Target_Left == 0 && Target_Right == 0)
    {
        Control_Reset_PI();
        Set_Pwm(0, 0);
    }
    else
    {
        Left_PWM = Incremental_PI_Left(Left_Speed, Target_Left);
        Right_PWM = Incremental_PI_Right(Right_Speed, Target_Right);
        Set_Pwm(Left_PWM, Right_PWM);
    }

    printf("T:%d,%d S:%d,%d PWM:%d,%d\r\n",
           Target_Left, Target_Right,
           Left_Speed, Right_Speed,
           Left_PWM, Right_PWM);
}

void Control_Tick_1ms(void)
{
    if (++control_millis >= CONTROL_PERIOD_MS)
    {
        control_millis = 0;
        control_ready = 1;
    }

    if (command_active != 0)
    {
        if (command_millis < CONTROL_TIMEOUT_MS)
        {
            command_millis++;
        }
        else
        {
            timeout_pending = 1;
        }
    }
}

void Control_Task(void)
{
    u8 command;

    /* USART interrupt stores the newest byte in USART_RX_CMD. */
    command = USART_RX_CMD;
    if (command != 0)
    {
        USART_RX_CMD = 0;
        Control_Process_Command(command);
    }

    if (timeout_pending != 0)
    {
        Control_Stop();
        printf("Command timeout; stopped\r\n");
    }

    if (control_ready != 0)
    {
        control_ready = 0;
        System_Control();
    }
}
