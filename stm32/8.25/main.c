#include "stm32f10x.h"
#include "sys.h"
#include "motor.h"

#define TEST_SPEED 4000

int main(void)
{
    /* ???STM32 */
    Stm32_Clock_Init(9);
    MY_NVIC_PriorityGroupConfig(2);
    uart_init(115200);

    /* ??ST-Link???SWD?? */
    JTAG_Set(JTAG_SWD_DISABLE);
    JTAG_Set(SWD_ENABLE);

    /* ???TIM4??PWM */
    PWM_Init(7199, 9);

    /* ??????2? */
    Car_Stop();
    delay_ms(2000);

    while (1)
    {
        /* 1. ??? */
        printf("FORWARD\r\n");
        Set_Pwm(TEST_SPEED, TEST_SPEED);
        delay_ms(2000);

        Car_Stop();
        delay_ms(1000);

        /* 2. ??? */
        printf("BACKWARD\r\n");
        Set_Pwm(-TEST_SPEED, -TEST_SPEED);
        delay_ms(2000);

        Car_Stop();
        delay_ms(1000);

        /* 3. ??:??????,??????? */
        printf("LEFT\r\n");
        Set_Pwm(0, TEST_SPEED);
        delay_ms(1500);

        Car_Stop();
        delay_ms(1000);

        /* 4. ??:????? */
        printf("RIGHT\r\n");
        Set_Pwm(TEST_SPEED, 0);
        delay_ms(1500);

        Car_Stop();
        delay_ms(1000);

        /* 5. ??????:???????? */
        printf("SPIN LEFT\r\n");
        Set_Pwm(-TEST_SPEED, TEST_SPEED);
        delay_ms(1500);

        Car_Stop();
        delay_ms(1000);

        /* 6. ?????? */
        printf("SPIN RIGHT\r\n");
        Set_Pwm(TEST_SPEED, -TEST_SPEED);
        delay_ms(1500);

        /* ??????,??3? */
        Car_Stop();
        delay_ms(3000);
    }
}
