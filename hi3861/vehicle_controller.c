#include <stdio.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "hi_time.h"
#include "hi_uart.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_uart.h"
#include "vehicle_protocol.h"
#include "vehicle_controller.h"

/* UART0 is reserved for CH340 debug logs; Bluetooth commands use UART1. */
#define DEBUG_UART_INDEX WIFI_IOT_UART_IDX_0
#define BLUETOOTH_UART_INDEX WIFI_IOT_UART_IDX_1
#define STM32_UART_INDEX WIFI_IOT_UART_IDX_2
#define BLUETOOTH_BAUD_RATE 9600
#define STM32_BAUD_RATE 115200
#define FRAME_PERIOD_US 100000ULL
#define SENSOR_PERIOD_US 50000ULL
#define DIGIT_IDLE_US 100000ULL
#define MAX_RUN_10THS 600U
#define STATUS_LINE_MAX 32U
#define BLUETOOTH_SPEED_LOW 100
#define BLUETOOTH_SPEED_HIGH 150
#define TRIG_GPIO 7
#define ECHO_GPIO 8
#define TRACE_LEFT_GPIO 13
#define TRACE_RIGHT_GPIO 14
#define OBSTACLE_CM 25.0f
#define AVOID_SPEED 100
#define AVOID_BACKUP_TICKS 10U  /* 10 x 50 ms = 500 ms */
#define AVOID_TURN_TICKS 14U    /* 14 x 50 ms = 700 ms */
#define AVOID_HIT_CONFIRM 2U
#define AVOID_STATUS_TICKS 20U  /* print distance about once per second */
#define EDGE_SPEED 100
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

typedef enum {
    AVOID_DRIVE_FORWARD = 0,
    AVOID_BACK_UP,
    AVOID_TURN
} AvoidState;

/* Power on in a safe stopped state.  Start autonomous edge guard explicitly
   with the Bluetooth G command after sensor polarity is checked. */
static volatile VehicleMode g_mode = VEHICLE_MODE_STOP;
static volatile int16_t g_left_target;
static volatile int16_t g_right_target;
static EdgeGuardState g_edge_state = EDGE_DRIVE_FORWARD;
static uint8_t g_edge_ticks;
static uint8_t g_turn_right;
static AvoidState g_avoid_state = AVOID_DRIVE_FORWARD;
static uint8_t g_avoid_ticks;
static uint8_t g_avoid_hits;
static uint8_t g_avoid_status_ticks;
static volatile uint8_t g_frame_requested;
static int16_t g_manual_speed = BLUETOOTH_SPEED_HIGH;
static unsigned long long g_run_deadline_us;

typedef enum {
    BLE_PARSE_NORMAL = 0,
    BLE_PARSE_STATUS_SECOND,
    BLE_PARSE_STATUS_REST
} BleParseState;

static BleParseState g_ble_parse_state;
static uint8_t g_status_count;
static char g_pending_motion;
static uint8_t g_pending_digits;
static unsigned int g_pending_duration;
static unsigned long long g_pending_last_byte_us;

static int16_t ClampTarget(int16_t value)
{
    if (value > 150) return 150;
    if (value < -150) return -150;
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
   not change the motor command.  Send Bluetooth command P while the car is on
   the tabletop before enabling edge guard with G. */
static void PrintTraceLevels(void)
{
    WifiIotGpioValue left_level;
    WifiIotGpioValue right_level;

    GpioGetInputVal(TRACE_LEFT_GPIO, &left_level);
    GpioGetInputVal(TRACE_RIGHT_GPIO, &right_level);
    printf("TRACE L=%d R=%d table=%d\r\n", (int)left_level,
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

/* Circle-arena avoid: drive forward, then back up and spin right when the
   front ultrasonic sees a wall.  Timeout is treated as blocked so a dead
   sensor does not drive into the wall. */
static void ObstacleAvoidControl(void)
{
    float distance = ReadDistanceCm();
    uint8_t blocked;

    blocked = (distance < 0.0f || distance < OBSTACLE_CM) ? 1U : 0U;

    if (g_avoid_state == AVOID_DRIVE_FORWARD) {
        if (g_avoid_status_ticks > 0U) g_avoid_status_ticks--;
        if (g_avoid_status_ticks == 0U) {
            if (distance < 0.0f) printf("AVOID dist=timeout state=fwd\r\n");
            else printf("AVOID dist=%d state=fwd\r\n", (int)distance);
            g_avoid_status_ticks = AVOID_STATUS_TICKS;
        }

        if (blocked == 0U) {
            g_avoid_hits = 0U;
            g_left_target = AVOID_SPEED;
            g_right_target = AVOID_SPEED;
            return;
        }

        if (g_avoid_hits < AVOID_HIT_CONFIRM) g_avoid_hits++;
        if (g_avoid_hits < AVOID_HIT_CONFIRM) {
            g_left_target = AVOID_SPEED;
            g_right_target = AVOID_SPEED;
            return;
        }

        g_avoid_hits = 0U;
        g_avoid_state = AVOID_BACK_UP;
        g_avoid_ticks = AVOID_BACKUP_TICKS;
        g_left_target = -AVOID_SPEED;
        g_right_target = -AVOID_SPEED;
        if (distance < 0.0f) printf("AVOID wall: timeout -> backup\r\n");
        else printf("AVOID wall: dist=%d -> backup\r\n", (int)distance);
        return;
    }

    if (g_avoid_state == AVOID_BACK_UP) {
        g_left_target = -AVOID_SPEED;
        g_right_target = -AVOID_SPEED;
        if (g_avoid_ticks > 0U) g_avoid_ticks--;
        if (g_avoid_ticks == 0U) {
            g_avoid_state = AVOID_TURN;
            g_avoid_ticks = AVOID_TURN_TICKS;
            printf("AVOID turn right\r\n");
        }
        return;
    }

    /* In-place right turn so a closed ring tends to keep circulating. */
    g_left_target = AVOID_SPEED;
    g_right_target = -AVOID_SPEED;
    if (g_avoid_ticks > 0U) g_avoid_ticks--;
    if (g_avoid_ticks == 0U) {
        g_avoid_state = AVOID_DRIVE_FORWARD;
        g_avoid_hits = 0U;
        g_avoid_status_ticks = 0U;
        printf("AVOID resume forward\r\n");
    }
}

static void SendControlFrame(void)
{
    static uint8_t sequence;
    uint8_t frame[VEHICLE_FRAME_SIZE];
    VehicleCommand command;

    command.command = VEHICLE_CMD_SET_WHEEL_TARGET;
    command.sequence = sequence++;
    command.left_target = g_left_target;
    command.right_target = g_right_target;
    VehicleProtocol_PackCommand(&command, frame);
    (void)UartWrite(STM32_UART_INDEX, frame, VEHICLE_FRAME_SIZE);
}

static void ApplyMotionCommand(char command)
{
    switch (command) {
        case 'w': Vehicle_SetManualCommand(g_manual_speed, g_manual_speed); break;
        case 's': case 'b': Vehicle_SetManualCommand(-g_manual_speed, -g_manual_speed); break;
        case 'a': case 'f': Vehicle_SetManualCommand(-50, g_manual_speed); break;
        case 'd': case 'c': Vehicle_SetManualCommand(g_manual_speed, -50); break;
        default: Vehicle_EmergencyStop(); break;
    }
}

/* Finish a pending direction letter plus its optional duration. */
static void ExecutePendingMotion(void)
{
    char command = g_pending_motion;
    unsigned int duration = g_pending_duration;
    uint8_t has_duration = g_pending_digits;

    g_pending_motion = 0;
    g_pending_digits = 0;
    g_pending_duration = 0;
    ApplyMotionCommand(command);

    if (has_duration != 0U && duration == 0U) {
        Vehicle_EmergencyStop();
        g_run_deadline_us = 0;
        printf("CMD %c: stop (time 0)\r\n", command);
    } else if (has_duration != 0U) {
        g_run_deadline_us = hi_get_us() + (unsigned long long)duration * 100000ULL;
        printf("CMD %c: %u x0.1s (L=%d R=%d)\r\n", command, duration,
               (int)g_left_target, (int)g_right_target);
    } else {
        g_run_deadline_us = 0;
        printf("CMD %c: hold (L=%d R=%d)\r\n", command,
               (int)g_left_target, (int)g_right_target);
    }
    g_frame_requested = 1U;
}

/* JDY-16 command parser.  Motion syntax is w/s/a/d plus optional tenths of
   a second (w50 = 5 s).  Entire +CONNECTED/+DISCONNECTED lines are consumed
   so their letters can never become motor commands. */
static void HandleControlByte(unsigned char byte)
{
    char command = (char)byte;

    if (g_pending_motion != 0) {
        if (command >= '0' && command <= '9') {
            g_pending_duration = g_pending_duration * 10U + (unsigned int)(command - '0');
            if (g_pending_duration > MAX_RUN_10THS) g_pending_duration = MAX_RUN_10THS;
            g_pending_digits++;
            g_pending_last_byte_us = hi_get_us();
            return;
        }
        ExecutePendingMotion();
    }

    if (g_ble_parse_state == BLE_PARSE_STATUS_REST) {
        if (command == '\n' || ++g_status_count >= STATUS_LINE_MAX) {
            g_ble_parse_state = BLE_PARSE_NORMAL;
        }
        return;
    }
    if (g_ble_parse_state == BLE_PARSE_STATUS_SECOND) {
        g_ble_parse_state = BLE_PARSE_STATUS_REST;
        g_status_count = 0;
        if (command == 'D') {
            Vehicle_EmergencyStop();
            g_run_deadline_us = 0;
            printf("BLE DISCONNECTED: auto stop\r\n");
        } else if (command == 'C') {
            Vehicle_EmergencyStop();
            printf("BLE CONNECTED\r\n");
        }
        return;
    }
    if (command == '+') {
        g_ble_parse_state = BLE_PARSE_STATUS_SECOND;
        return;
    }

    if (command >= 'A' && command <= 'Z') command = (char)(command - 'A' + 'a');
    switch (command) {
        case 'w': case 's': case 'a': case 'd':
        case 'b': case 'c': case 'f':
            g_pending_motion = command;
            g_pending_digits = 0;
            g_pending_duration = 0;
            g_pending_last_byte_us = hi_get_us();
            break;
        case '0': case 'x': case 'o': case 'e':
            Vehicle_EmergencyStop();
            g_run_deadline_us = 0;
            printf("CMD stop\r\n");
            break;
        case 'i':
            g_manual_speed = BLUETOOTH_SPEED_LOW;
            printf("CMD speed=%d\r\n", (int)g_manual_speed);
            break;
        case 'k':
            g_manual_speed = BLUETOOTH_SPEED_HIGH;
            printf("CMD speed=%d\r\n", (int)g_manual_speed);
            break;
        case 'g':
            Vehicle_SetMode(VEHICLE_MODE_EDGE_GUARD);
            printf("CMD edge guard\r\n");
            break;
        case 'v':
            Vehicle_SetMode(VEHICLE_MODE_OBSTACLE_AVOID);
            printf("CMD obstacle avoid\r\n");
            break;
        case 'p': PrintTraceLevels(); break;
        case '\r': case '\n': case ' ': break;
        default: printf("CMD? 0x%02X\r\n", (unsigned int)byte); break;
    }
}

/* One task reads Bluetooth, updates autonomous modes and sends the teacher's
   six-byte motor frame over hardware UART2. */
static void VehicleMainTask(void)
{
    unsigned char byte;
    unsigned long long now;
    unsigned long long last_sensor_us = 0;
    unsigned long long last_frame_us = 0;
    uint8_t loop_confirmed = 0;
    hi_s32 read_length;

    printf("vehicle main task started: BLE UART1 9600 + STM32 UART2 115200\r\n");
    while (1) {
        /* Use the SDK timeout API explicitly.  The wrapper's default RX
           blocking mode differs across Hi3861 SDK configurations; an
           unbounded read here would also stop the GPIO11 heartbeat. */
        read_length = hi_uart_read_timeout(HI_UART_IDX_1, &byte, 1, 1);
        if (read_length == 1) {
            printf("BLE RX byte=0x%02X\r\n", (unsigned int)byte);
            HandleControlByte(byte);
        }
        if (loop_confirmed == 0U) {
            printf("vehicle main loop active: UART1 timed read returned\r\n");
            loop_confirmed = 1U;
        }

        if (g_pending_motion != 0 &&
            hi_get_us() - g_pending_last_byte_us >= DIGIT_IDLE_US) {
            ExecutePendingMotion();
        }
        if (g_run_deadline_us != 0 && hi_get_us() >= g_run_deadline_us) {
            Vehicle_EmergencyStop();
            g_run_deadline_us = 0;
            printf("AUTO STOP\r\n");
        }

        now = hi_get_us();
        if (now - last_sensor_us >= SENSOR_PERIOD_US) {
            if (g_mode == VEHICLE_MODE_OBSTACLE_AVOID) {
                ObstacleAvoidControl();
            } else if (g_mode == VEHICLE_MODE_EDGE_GUARD) {
                EdgeGuardControl();
            }
            last_sensor_us = now;
        }

        if (g_frame_requested != 0U || now - last_frame_us >= FRAME_PERIOD_US) {
            SendControlFrame();
            g_frame_requested = 0U;
            last_frame_us = hi_get_us();
        }
        osDelay(1);
    }
}

void Vehicle_SetMode(VehicleMode mode)
{
    g_mode = mode;
    g_edge_state = EDGE_DRIVE_FORWARD;
    g_edge_ticks = 0U;
    g_avoid_state = AVOID_DRIVE_FORWARD;
    g_avoid_ticks = 0U;
    g_avoid_hits = 0U;
    g_avoid_status_ticks = 0U;
    if (mode == VEHICLE_MODE_STOP) Vehicle_EmergencyStop();
    g_frame_requested = 1U;
}

void Vehicle_SetManualCommand(int16_t left, int16_t right)
{
    g_left_target = ClampTarget(left);
    g_right_target = ClampTarget(right);
    g_mode = VEHICLE_MODE_MANUAL;
    g_frame_requested = 1U;
}

void Vehicle_EmergencyStop(void)
{
    g_left_target = 0;
    g_right_target = 0;
    g_mode = VEHICLE_MODE_STOP;
    g_frame_requested = 1U;
}

static void VehicleAppInit(void)
{
    WifiIotUartAttribute debug_uart = {115200, 8, 1, 0, 0};
    hi_uart_attribute bluetooth_uart = {BLUETOOTH_BAUD_RATE, 8, 1, 0, 0};
    hi_uart_attribute stm32_uart = {STM32_BAUD_RATE, 8, 1, 0, 0};
    osThreadAttr_t attr = {0};
    unsigned int uart0_status;
    hi_u32 uart1_status;
    hi_u32 uart2_status;

    GpioInit();
    /* CH340 debug log: UART0 on GPIO3/GPIO4. */
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_3, WIFI_IOT_IO_FUNC_GPIO_3_UART0_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_4, WIFI_IOT_IO_FUNC_GPIO_4_UART0_RXD);
    uart0_status = UartInit(DEBUG_UART_INDEX, &debug_uart, NULL);
    /* Own UART1 explicitly: app_main may have initialized it before the app. */
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_0, WIFI_IOT_IO_FUNC_GPIO_0_UART1_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_1, WIFI_IOT_IO_FUNC_GPIO_1_UART1_RXD);
    (void)hi_uart_deinit(HI_UART_IDX_1);
    uart1_status = hi_uart_init(HI_UART_IDX_1, &bluetooth_uart, HI_NULL);
    /* Teacher reference link: GPIO11/12 = UART2, 115200 8N1. */
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);
    (void)hi_uart_deinit(HI_UART_IDX_2);
    uart2_status = hi_uart_init(HI_UART_IDX_2, &stm32_uart, HI_NULL);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_7, WIFI_IOT_IO_FUNC_GPIO_7_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_8, WIFI_IOT_IO_FUNC_GPIO_8_GPIO);
    GpioSetDir(TRIG_GPIO, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetDir(ECHO_GPIO, WIFI_IOT_GPIO_DIR_IN);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(TRACE_LEFT_GPIO, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(TRACE_RIGHT_GPIO, WIFI_IOT_GPIO_DIR_IN);

    printf("vehicle app init: uart0=%u uart1=%u@9600 uart2=%u@115200 frame=FC..FD\r\n",
           uart0_status, uart1_status, uart2_status);

    attr.name = "vehicle_main";
    attr.stack_size = 4096;
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)VehicleMainTask, NULL, &attr) == NULL)
        printf("vehicle main task create failed\r\n");
}
APP_FEATURE_INIT(VehicleAppInit);
