#include "colorful_led.h"
#include "delay.h"
#include "vehicle_ws2812.h"

#define LED_COUNT 6
#define TURN_DELTA 80
#define TURN_BLINK_MS 400
#define RIGHT_LED_INDEX 1
#define LEFT_LED_INDEX  6

static void SetAllOff(void)
{
    u8 i;
    for (i = 1; i <= LED_COUNT; ++i) {
        L_ws2812_rgb(i, WS_DARK);
        R_ws2812_rgb(i, WS_DARK);
    }
}

void VehicleLights_Init(void)
{
    colorful_led_Init();
    SetAllOff();
    L_ws2812_refresh(LED_COUNT);
    R_ws2812_refresh(LED_COUNT);
}

void VehicleLights_BootSelfTest(void)
{
    u8 i;
    for (i = 1; i <= LED_COUNT; ++i) {
        L_ws2812_rgb(i, WS_GREEN);
        R_ws2812_rgb(i, WS_GREEN);
    }
    L_ws2812_refresh(LED_COUNT);
    R_ws2812_refresh(LED_COUNT);
    delay_ms(1000);
    SetAllOff();
    L_ws2812_refresh(LED_COUNT);
    R_ws2812_refresh(LED_COUNT);
}

void VehicleLights_Update(int left_target, int right_target, u16 elapsed_ms)
{
    static u16 blink_ms;
    static u8 blink_on;
    u8 index = 0;

    if (left_target + TURN_DELTA < right_target) index = LEFT_LED_INDEX;
    else if (right_target + TURN_DELTA < left_target) index = RIGHT_LED_INDEX;
    else { blink_ms = 0; blink_on = 0; }
    if (index != 0) {
        blink_ms += elapsed_ms;
        if (blink_ms >= TURN_BLINK_MS) { blink_ms = 0; blink_on = !blink_on; }
    }
    SetAllOff();
    if (blink_on) {
        L_ws2812_rgb(index, WS_YELLOW);
        R_ws2812_rgb(index, WS_YELLOW);
    }
    L_ws2812_refresh(LED_COUNT);
    R_ws2812_refresh(LED_COUNT);
}
