#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include <stdint.h>

typedef enum {
    VEHICLE_MODE_STOP = 0,
    VEHICLE_MODE_MANUAL,
    VEHICLE_MODE_OBSTACLE_AVOID,
    VEHICLE_MODE_EDGE_GUARD
} VehicleMode;

void Vehicle_SetMode(VehicleMode mode);
void Vehicle_SetManualCommand(int16_t left, int16_t right);
void Vehicle_EmergencyStop(void);

#endif
