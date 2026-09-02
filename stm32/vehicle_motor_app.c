#include "vehicle_motor_app.h"
#include "vehicle_protocol.h"
#include "motor.h"
#include "encoder.h"
#include "vehicle_ws2812.h"

#define COMMAND_TIMEOUT_MS 200
#define TARGET_LIMIT 150
#define PWM_LIMIT 7199
#define VEHICLE_LEFT_SIGN  1
#define VEHICLE_RIGHT_SIGN 1

typedef struct { int target; int integral; int last_error; } WheelPid;
static volatile u16 g_command_age_ms = COMMAND_TIMEOUT_MS + 1;
static volatile WheelPid g_left;
static volatile WheelPid g_right;

static int Clamp(int value, int lower, int upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

static int PidCalculate(volatile WheelPid *pid, int actual)
{
    int error = pid->target - actual;
    int output;

    if (pid->target == 0) {
        pid->integral = 0;
        pid->last_error = 0;
        return 0;
    }

    /* The original chassis needs roughly 4000/7199 PWM to overcome static
       friction.  Add signed feed-forward, then let PID correct the speed. */
    pid->integral = Clamp(pid->integral + error, -8000, 8000);
    output = (pid->target > 0 ? 3600 : -3600) +
             18 * error + pid->integral / 8 +
             4 * (error - pid->last_error);
    pid->last_error = error;
    return Clamp(output, -PWM_LIMIT, PWM_LIMIT);
}

void VehicleMotor_Init(void)
{
    PWM_Init(7199, 9);
    Encoder_Init_TIM2();
    Encoder_Init_TIM3();
    VehicleLights_Init();
    Car_Stop();
}

void VehicleMotor_OnRxByte(u8 byte)
{
    VehicleCommand command;
    if (!VehicleProtocol_ParseByte(byte, &command)) return;
    if (command.command == VEHICLE_CMD_ESTOP) {
        g_left.target = 0;
        g_right.target = 0;
        g_left.integral = 0;
        g_right.integral = 0;
        Car_Stop();
    } else if (command.command == VEHICLE_CMD_SET_WHEEL_TARGET) {
        g_left.target = Clamp(command.left_target, -TARGET_LIMIT, TARGET_LIMIT);
        g_right.target = Clamp(command.right_target, -TARGET_LIMIT, TARGET_LIMIT);
    } else return;
    g_command_age_ms = 0;
}

void VehicleMotor_1msTick(void)
{
    if (g_command_age_ms <= COMMAND_TIMEOUT_MS) g_command_age_ms++;
}

void VehicleMotor_Control20ms(void)
{
    int left_actual;
    int right_actual;
    if (g_command_age_ms > COMMAND_TIMEOUT_MS) {
        g_left.target = 0;
        g_right.target = 0;
        g_left.integral = 0;
        g_right.integral = 0;
        Car_Stop();
        VehicleLights_Update(0, 0, 20);
        return;
    }
    left_actual = VEHICLE_LEFT_SIGN * Read_Encoder(2);
    right_actual = VEHICLE_RIGHT_SIGN * Read_Encoder(3);
    Set_Pwm(PidCalculate(&g_left, left_actual), PidCalculate(&g_right, right_actual));
    VehicleLights_Update(g_left.target, g_right.target, 20);
}
