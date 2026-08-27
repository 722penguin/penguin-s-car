#include "stm32f10x.h"
#include "sys.h"
#include "encoder.h"
#include <stdio.h>

/* ??100ms????????????? */
int left_count = 0;
int right_count = 0;

/* SysTick?1ms???? */
volatile u32 millis = 0;

/* ??100ms??1,??main????? */
volatile u8 speed_sample_ready = 0;

int main(void)
{
    /* ???????:72MHz */
    Stm32_Clock_Init(9);

    /* ????????? */
    MY_NVIC_PriorityGroupConfig(2);

    /* ?????,???115200 */
    uart_init(115200);

    /* ??JTAG,??SWD???? */
    JTAG_Set(JTAG_SWD_DISABLE);
    JTAG_Set(SWD_ENABLE);

    /* ????????? */
    Encoder_Init_TIM2();
    Encoder_Init_TIM3();

    /*
     * ?????72MHz
     * 72000000 / 1000???1ms????SysTick??
     */
    SysTick_Config(72000000 / 1000);

    printf("encoder test start\r\n");

    while (1)
    {
        /* ??100ms????????? */
        if (speed_sample_ready == 1)
        {
            speed_sample_ready = 0;

            left_count = Read_Encoder(2);
            right_count = Read_Encoder(3);

            printf("left count  : %d\r\n", left_count);
            printf("right count : %d\r\n", right_count);
        }
    }
}
