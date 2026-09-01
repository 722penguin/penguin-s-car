#ifndef VEHICLE_PROTOCOL_H
#define VEHICLE_PROTOCOL_H

#include <stdint.h>

#define VEHICLE_SOF0 0xAA
#define VEHICLE_SOF1 0x55
#define VEHICLE_EOF0 0x0D
#define VEHICLE_EOF1 0x0A
#define VEHICLE_CMD_SET_WHEEL_TARGET 0x01
#define VEHICLE_CMD_ESTOP            0x02
#define VEHICLE_FRAME_SIZE 11

typedef struct {
    uint8_t command;
    uint8_t sequence;
    int16_t left_target;
    int16_t right_target;
} VehicleCommand;

uint8_t VehicleProtocol_Xor(const uint8_t *data, uint8_t length);
uint8_t VehicleProtocol_PackCommand(const VehicleCommand *command, uint8_t *frame);

#endif
