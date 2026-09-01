#ifndef VEHICLE_MOTOR_APP_H
#define VEHICLE_MOTOR_APP_H

#include "sys.h"

void VehicleMotor_Init(void);
void VehicleMotor_Control20ms(void);
void VehicleMotor_OnRxByte(u8 byte);
void VehicleMotor_1msTick(void);

#endif
