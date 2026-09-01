#include <stdio.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "hi_time.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_uart.h"
#include "vehicle_protocol.h"
#include "vehicle_controller.h"

#define UART_INDEX WIFI_IOT_UART_IDX_2
#define CONSOLE_UART_INDEX WIFI_IOT_UART_IDX_0
#define TRIG_GPIO 7
#define ECHO_GPIO 8
#define TRACE_LEFT_GPIO 13
#define TRACE_RIGHT_GPIO 14
#define CONTROL_PERIOD_MS 50
#define OBSTACLE_CM 20.0f
#define EDGE_SPEED 220
#define EDGE_BACKUP_TICKS 8U  /* 8 x 50 ms = 400 ms */
#define EDGE_TURN_TICKS 10U   /* 10 x 50 ms = 500 ms */

/* The original underbody tracking probes read LOW on the white tabletop and
   HIGH once they no longer see a reflected surface.  Reverse this value only
   if the serial diagnostic proves this particular car uses opposite logic. */
#define TABLE_PRESENT_LEVEL WIFI_IOT_GPIO_VALUE0

typedef enum {
    EDGE_DRIVE_FORWARD = 0,
    EDGE_BACK_UP,
    EDGE_TURN_AWAY
} EdgeGuardState;

/* Autonomous edge-guard mode starts automatically after power-on.  It does
   not need a PC, a USB serial connection, or a console command to continue. */
static volatile VehicleMode g_mode = VEHICLE_MODE_EDGE_GUARD;
static volatile int16_t g_left_target;
static volatile int16_t g_right_target;
static volatile uint8_t g_estop_requested;
static EdgeGuardState g_edge_state = EDGE_DRIVE_FORWARD;
static uint8_t g_edge_ticks;
static uint8_t g_turn_right;

static int16_t ClampTarget(int16_t value)
{
    if (value > 1000) return 1000;
    if (value < -1000) return -1000;
    return value;
}

/* 返回 -1 表示传感器超时；绝不在这里无限等待。 */
static float ReadDistanceCm(void)
{
    WifiIotGpioValue level;
    unsigned long long start;
    unsigned long long now;

    GpioSetOutputVal(TRIG_GPIO, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(15);
    GpioSetOutputVal(TRIG_GPIO, WIFI_IOT_GPIO_VALUE0);

    start = hi_get_us();
    do {
        GpioGetInputVal(ECHO_GPIO, &level);
        if (level == WIFI_IOT_GPIO_VALUE1) break;
    } while (hi_get_us() - start < 30000);
    if (level != WIFI_IOT_GPIO_VALUE1) return -1.0f;

    start = hi_get_us();
    do {
        GpioGetInputVal(ECHO_GPIO, &level);
        now = hi_get_us();
        if (level == WIFI_IOT_GPIO_VALUE0) return (float)(now - start) * 0.017f;
    } while (hi_get_us() - start < 30000);
    return -1.0f;
}

static uint8_t IsTablePresent(WifiIotGpioValue value)
{
    return (value == TABLE_PRESENT_LEVEL) ? 1U : 0U;
}

/* Safe diagnostic: it only prints the two underbody probe levels and does
   not change the motor command.  Use console command P while the car is on
   the tabletop before enabling edge guard with E. */
static void PrintTraceLevels(void)
{
    WifiIotGpioValue left_level;
    WifiIotGpioValue right_level;

    GpioGetInputVal(TRACE_LEFT_GPIO, &left_level);
    GpioGetInputVal(TRACE_RIGHT_GPIO, &right_level);
    printf("TRACE L=%d R=%d table=%d\\r\\n", (int)left_level,
           (int)right_level, (int)TABLE_PRESENT_LEVEL);
}

/* Run at 50 ms intervals.  The state machine never blocks sensor polling:
   first retreat from the edge, then spin toward the side that still has a
   table surface, and finally resume forward motion. */
static void EdgeGuardControl(void)
{
    WifiIotGpioValue left_level;
    WifiIotGpioValue right_level;
    uint8_t left_on_table;
    uint8_t right_on_table;

    GpioGetInputVal(TRACE_LEFT_GPIO, &left_level);
    GpioGetInputVal(TRACE_RIGHT_GPIO, &right_level);
    left_on_table = IsTablePresent(left_level);
    right_on_table = IsTablePresent(right_level);

    if (g_edge_state == EDGE_DRIVE_FORWARD) {
        if (left_on_table != 0U && right_on_table != 0U) {
            g_left_target = EDGE_SPEED;
            g_right_target = EDGE_SPEED;
            return;
        }

        /* Left probe over the edge -> pivot right; right probe -> pivot left.
           With both probes over the edge, choose right deterministically. */
        g_turn_right = (left_on_table == 0U) ? 1U : 0U;
        g_edge_state = EDGE_BACK_UP;
        g_edge_ticks = EDGE_BACKUP_TICKS;
        g_left_target = -EDGE_SPEED;
        g_right_target = -EDGE_SPEED;
        return;
    }

    if (g_edge_state == EDGE_BACK_UP) {
        g_left_target = -EDGE_SPEED;
        g_right_target = -EDGE_SPEED;
        if (g_edge_ticks > 0U) g_edge_ticks--;
        if (g_edge_ticks == 0U) {
            g_edge_state = EDGE_TURN_AWAY;
            g_edge_ticks = EDGE_TURN_TICKS;
        }
        return;
    }

    if (g_turn_right != 0U) {
        g_left_target = EDGE_SPEED;
        g_right_target = -EDGE_SPEED;
    } else {
        g_left_target = -EDGE_SPEED;
        g_right_target = EDGE_SPEED;
    }
    if (g_edge_ticks > 0U) g_edge_ticks--;
    if (g_edge_ticks == 0U) g_edge_state = EDGE_DRIVE_FORWARD;
}

static void SendControlFrame(void)
{
    static uint8_t sequence;
    uint8_t frame[VEHICLE_FRAME_SIZE];
    VehicleCommand command;

    command.command = g_estop_requested ? VEHICLE_CMD_ESTOP : VEHICLE_CMD_SET_WHEEL_TARGET;
    command.sequence = sequence++;
    command.left_target = g_estop_requested ? 0 : g_left_target;
    command.right_target = g_estop_requested ? 0 : g_right_target;
    VehicleProtocol_PackCommand(&command, frame);
    UartWrite(UART_INDEX, frame, VEHICLE_FRAME_SIZE);
    g_estop_requested = 0;
}

static void VehicleControlTask(void)
{
    while (1) {
        if (g_mode == VEHICLE_MODE_OBSTACLE_AVOID) {
            float distance = ReadDistanceCm();
            /* 无有效距离同样停车，安全优先。 */
            if (distance < 0.0f || distance < OBSTACLE_CM) {
                g_left_target = 0;
                g_right_target = 0;
            } else {
                g_left_target = 300;
                g_right_target = 300;
            }
        }
        else if (g_mode == VEHICLE_MODE_EDGE_GUARD) {
            EdgeGuardControl();
        }
        SendControlFrame();
        osDelay(CONTROL_PERIOD_MS);
    }
}

/* CH340 串口助手命令：W/F 前进，S 后退，A 左转，D 右转，X 停止，
   O 前方避障，E 防掉桌，P 只打印车底红外电平。 */
static void VehicleConsoleTask(void)
{
    unsigned char byte;
    while (1) {
        if (UartRead(CONSOLE_UART_INDEX, &byte, 1) == 1) {
            switch (byte) {
                case 'W': case 'w': case 'F': case 'f':
                    Vehicle_SetManualCommand(300, 300); break;
                case 'S': case 's': Vehicle_SetManualCommand(-300, -300); break;
                case 'A': case 'a': Vehicle_SetManualCommand(-220, 220); break;
                case 'D': case 'd': Vehicle_SetManualCommand(220, -220); break;
                case 'X': case 'x': Vehicle_EmergencyStop(); break;
                case 'O': case 'o': Vehicle_SetMode(VEHICLE_MODE_OBSTACLE_AVOID); break;
                case 'E': case 'e': Vehicle_SetMode(VEHICLE_MODE_EDGE_GUARD); break;
                case 'P': case 'p': PrintTraceLevels(); break;
                default: break;
            }
        } else {
            osDelay(10);
        }
    }
}

void Vehicle_SetMode(VehicleMode mode)
{
    g_mode = mode;
    g_edge_state = EDGE_DRIVE_FORWARD;
    g_edge_ticks = 0U;
    if (mode == VEHICLE_MODE_STOP) Vehicle_EmergencyStop();
}

void Vehicle_SetManualCommand(int16_t left, int16_t right)
{
    g_left_target = ClampTarget(left);
    g_right_target = ClampTarget(right);
    g_mode = VEHICLE_MODE_MANUAL;
}

void Vehicle_EmergencyStop(void)
{
    g_left_target = 0;
    g_right_target = 0;
    g_mode = VEHICLE_MODE_STOP;
    g_estop_requested = 1;
}

static void VehicleAppInit(void)
{
    WifiIotUartAttribute uart = {115200, 8, 1, 0, 0};
    osThreadAttr_t attr = {0};

    GpioInit();
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);
    UartInit(UART_INDEX, &uart, NULL);
    /* 原理图：CH340 使用 IO3/IO4，即 UART0。 */
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_3, WIFI_IOT_IO_FUNC_GPIO_3_UART0_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_4, WIFI_IOT_IO_FUNC_GPIO_4_UART0_RXD);
    UartInit(CONSOLE_UART_INDEX, &uart, NULL);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_7, WIFI_IOT_IO_FUNC_GPIO_7_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_8, WIFI_IOT_IO_FUNC_GPIO_8_GPIO);
    GpioSetDir(TRIG_GPIO, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetDir(ECHO_GPIO, WIFI_IOT_GPIO_DIR_IN);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(TRACE_LEFT_GPIO, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(TRACE_RIGHT_GPIO, WIFI_IOT_GPIO_DIR_IN);

    attr.name = "vehicle_ctrl";
    attr.stack_size = 4096;
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)VehicleControlTask, NULL, &attr) == NULL)
        printf("vehicle task create failed\r\n");
    attr.name = "vehicle_console";
    if (osThreadNew((osThreadFunc_t)VehicleConsoleTask, NULL, &attr) == NULL)
        printf("console task create failed\r\n");
}
APP_FEATURE_INIT(VehicleAppInit);
