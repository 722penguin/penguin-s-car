#ifndef VEHICLE_PROTOCOL_H
#define VEHICLE_PROTOCOL_H

#include "sys.h"

#define VEHICLE_SOF0 0xAA
#define VEHICLE_SOF1 0x55
#define VEHICLE_EOF0 0x0D
#define VEHICLE_EOF1 0x0A
#define VEHICLE_CMD_SET_WHEEL_TARGET 0x01
#define VEHICLE_CMD_ESTOP 0x02
#define VEHICLE_FRAME_SIZE 11

typedef struct {
    u8 command;
    u8 sequence;
    short left_target;
    short right_target;
} VehicleCommand;

u8 VehicleProtocol_Xor(const u8 *data, u8 length);
u8 VehicleProtocol_ParseByte(u8 byte, VehicleCommand *command);

#endif
