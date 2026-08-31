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
static volatile u8 direct_pwm_active = 0;
static int direct_left_pwm = 0;
static int direct_right_pwm = 0;

#define COMMAND_FRAME_MAX 8U
#define COMMAND_TEXT_MAX  16U
static u8 frame_state = 0;
static u8 frame_length = 0;
static u8 frame_index = 0;
static u8 frame_type = 0;
static u8 frame_checksum = 0;
static u8 frame_payload[COMMAND_FRAME_MAX];
/* QST Hi3861 -> STM32 motor frame: FC, Ldir, Lspeed, Rdir, Rspeed, FD. */
static u8 qst_frame_state = 0;
static u8 qst_frame_index = 0;
static u8 qst_frame_payload[4];
static char command_text[COMMAND_TEXT_MAX];
static u8 command_length = 0;
static volatile u8 command_text_idle_ms = 0;
static volatile u8 command_text_pending = 0;

static void Control_Set_Direct_PWM(int left_percent, int right_percent);

/*
 * Text command format: left_percent,right_percent
 * Examples: "80,80" moves forward and "0,0" stops.
 * The command is accepted after a short idle interval; no line ending is required.
 */
static u8 Parse_Speed_Pair(const char *text, int *left, int *right)
{
    int values[2] = {0, 0};
    u8 index = 0;
    u8 has_digit = 0;
    u8 negative = 0;

    while (*text != '\0')
    {
        char ch = *text++;
        if (ch >= '0' && ch <= '9')
        {
            if (values[index] > 300) return 0;
            values[index] = values[index] * 10 + (ch - '0');
            has_digit = 1;
        }
        else if (ch == '-' && !has_digit && !negative)
        {
            negative = 1;
        }
        else if (ch == ',' && index == 0U && has_digit)
        {
            if (negative) values[index] = -values[index];
            index = 1U;
            has_digit = 0;
            negative = 0;
        }
        else return 0;
    }

    if (index != 1U || !has_digit) return 0;
    if (negative) values[1] = -values[1];
    *left = values[0];
    *right = values[1];
    return 1;
}

static void Control_Process_Text(void)
{
    int left;
    int right;

    if (command_length == 0U) return;
    command_text[command_length] = '\0';
    if (Parse_Speed_Pair(command_text, &left, &right) != 0)
    {
        Control_Set_Direct_PWM(left, right);
        printf("ACK %d,%d\r\n", left, right);
    }
    else
    {
        Control_Stop();
        printf("ERR bad command; stopped\r\n");
    }
    command_length = 0U;
    command_text_pending = 0U;
}

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

/* USB command values are percentages (-100..100), mapped to safe PWM. */
static void Control_Set_Direct_PWM(int left_percent, int right_percent)
{
    if (left_percent > CONTROL_MANUAL_LIMIT) left_percent = CONTROL_MANUAL_LIMIT;
    if (left_percent < -CONTROL_MANUAL_LIMIT) left_percent = -CONTROL_MANUAL_LIMIT;
    if (right_percent > CONTROL_MANUAL_LIMIT) right_percent = CONTROL_MANUAL_LIMIT;
    if (right_percent < -CONTROL_MANUAL_LIMIT) right_percent = -CONTROL_MANUAL_LIMIT;
    direct_left_pwm = left_percent * CONTROL_MANUAL_PWM_SCALE;
    direct_right_pwm = right_percent * CONTROL_MANUAL_PWM_SCALE;
    direct_pwm_active = 1U;
    command_active = 1U;
    command_millis = 0U;
    timeout_pending = 0U;
    Control_Reset_PI();
    Set_Pwm(direct_left_pwm, direct_right_pwm);
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

    direct_pwm_active = 0U;

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
    direct_pwm_active = 0U;

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

void Control_Process_UartByte(u8 byte)
{
    int left;
    int right;

    /* The USB port on this car belongs to Hi3861.  Motor commands reach
       STM32 through Hi3861 UART2 using this six-byte QST frame. */
    if (qst_frame_state != 0U)
    {
        if (qst_frame_index < 4U)
        {
            qst_frame_payload[qst_frame_index++] = byte;
            return;
        }
        qst_frame_state = 0U;
        if (byte == 0xFDU && qst_frame_payload[0] <= 1U && qst_frame_payload[2] <= 1U)
        {
            left = (int)qst_frame_payload[1];
            right = (int)qst_frame_payload[3];
            if (qst_frame_payload[0] != 0U) left = -left;
            if (qst_frame_payload[2] != 0U) right = -right;
            Control_Set_Target(left, right);
        }
        else Control_Stop();
        return;
    }
    if (byte == 0xFCU)
    {
        qst_frame_state = 1U;
        qst_frame_index = 0U;
        return;
    }
    if (frame_state == 0U)
    {
        if (byte == USART1_FRAME_HEAD1) frame_state = 1U;
    }
    else if (frame_state == 1U)
    {
        if (byte == USART1_FRAME_HEAD2) frame_state = 2U;
        else frame_state = (byte == USART1_FRAME_HEAD1) ? 1U : 0U;
    }
    else if (frame_state == 2U)
    {
        frame_length = byte;
        frame_checksum = byte;
        frame_state = (frame_length <= COMMAND_FRAME_MAX) ? 3U : 0U;
    }
    else if (frame_state == 3U)
    {
        frame_type = byte;
        frame_checksum = (u8)(frame_checksum + byte);
        frame_index = 0U;
        frame_state = (frame_length == 0U) ? 5U : 4U;
    }
    else if (frame_state == 4U)
    {
        frame_payload[frame_index++] = byte;
        frame_checksum = (u8)(frame_checksum + byte);
        if (frame_index >= frame_length) frame_state = 5U;
    }
    else
    {
        frame_state = 0U;
        if (frame_checksum == byte && frame_type == USART1_FRAME_SPEED && frame_length == 4U)
        {
            left = (int)((u16)frame_payload[0] | ((u16)frame_payload[1] << 8));
            right = (int)((u16)frame_payload[2] | ((u16)frame_payload[3] << 8));
            if (left >= -300 && left <= 300 && right >= -300 && right <= 300)
            {
                Control_Set_Target(left, right);
            }
            else Control_Stop();
        }
        else Control_Stop();
        return;
    }

    if (frame_state != 0U) return;
    if (byte == '\r') return;
    if (byte == '\n')
    {
        Control_Process_Text();
        return;
    }
    if (command_length < (COMMAND_TEXT_MAX - 1U))
    {
        command_text[command_length++] = (char)byte;
        command_text_idle_ms = 0U;
        command_text_pending = 1U;
    }
    else
    {
        command_length = 0;
        Control_Stop();
        printf("ERR command too long; stopped\r\n");
    }
}

void System_Control(void)
{
    Left_Speed = Read_Encoder(2);
    Right_Speed = Read_Encoder(3);

    if (direct_pwm_active != 0U)
    {
        Set_Pwm(direct_left_pwm, direct_right_pwm);
        return;
    }
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

    if (command_text_pending != 0U && command_text_idle_ms < CONTROL_TEXT_IDLE_MS)
    {
        command_text_idle_ms++;
    }
}

void Control_Task(void)
{
    u8 byte;
    u8 uart_overflow;
    u8 uart_error;
    while (USART1_ReadByte(&byte) != 0)
    {
        Control_Process_UartByte(byte);
    }
    if (command_text_pending != 0U && command_text_idle_ms >= CONTROL_TEXT_IDLE_MS)
    {
        Control_Process_Text();
    }
    uart_overflow = USART1_TakeRxOverflow();
    uart_error = USART1_TakeError();
    if (uart_overflow != 0U || uart_error != 0U)
    {
        frame_state = 0U;
        Control_Stop();
    }

    if (timeout_pending != 0)
    {
        Control_Stop();
    }

    if (control_ready != 0)
    {
        control_ready = 0;
        System_Control();
    }
}
