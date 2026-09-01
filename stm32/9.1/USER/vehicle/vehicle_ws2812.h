#ifndef VEHICLE_WS2812_H
#define VEHICLE_WS2812_H

#include "sys.h"

void VehicleLights_Init(void);
void VehicleLights_BootSelfTest(void);
/* 根据差速命令更新左右转向灯：左慢于右为左转，反之为右转。 */
void VehicleLights_Update(int left_target, int right_target, u16 elapsed_ms);

#endif
