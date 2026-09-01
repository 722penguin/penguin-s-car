#include "sys.h"
#include "delay.h"
#include "usart_vehicle.h"
#include "vehicle_motor_app.h"

int main(void)
{
    u8 divider = 0;
    Stm32_Clock_Init(9);
    MY_NVIC_PriorityGroupConfig(2);
    delay_init();
    VehicleUart_Init(115200);
    VehicleMotor_Init();
    VehicleLights_BootSelfTest();

    while (1) {
        delay_ms(1);
        VehicleMotor_1msTick();
        if (++divider >= 20) {
            divider = 0;
            VehicleMotor_Control20ms();
        }
    }
}
