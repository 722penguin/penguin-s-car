#include "stm32f10x.h"
#include "sys.h"
#include "encoder.h"
#include "motor.h"
#include "control_systems.h"
#include <stdio.h>

int main(void)
{
    /* STM32 system clock: 72 MHz. */
    Stm32_Clock_Init(9);
    MY_NVIC_PriorityGroupConfig(2);

    /* USART1: 115200, 8 data bits, even parity, 1 stop bit. */
    uart_init(115200);

    /* Keep SWD available for downloading and debugging. */
    JTAG_Set(JTAG_SWD_DISABLE);
    JTAG_Set(SWD_ENABLE);

    /* Left/right wheel encoders and motor PWM. */
    Encoder_Init_TIM2();
    Encoder_Init_TIM3();
    PWM_Init(7199, 9);

    /* SysTick interrupt every 1 ms. */
    SysTick_Config(72000000 / 1000);

    /* Safety first: power-on state is stopped. */
    Control_Stop();

    while (1)
    {
        /* Handles serial commands and runs the 100 ms PI control task. */
        Control_Task();
    }
}
