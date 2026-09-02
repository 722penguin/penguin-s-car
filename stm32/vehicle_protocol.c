#include "vehicle_protocol.h"

u8 VehicleProtocol_Xor(const u8 *data, u8 length)
{
    u8 value = 0;
    while (length-- > 0) value ^= *data++;
    return value;
}

u8 VehicleProtocol_ParseByte(u8 byte, VehicleCommand *command)
{
    static u8 buffer[VEHICLE_FRAME_SIZE];
    static u8 index;

    if (index == 0 && byte != VEHICLE_SOF0) return 0;
    buffer[index++] = byte;
    if (index < VEHICLE_FRAME_SIZE) return 0;
    index = 0;
    if (buffer[5] != VEHICLE_EOF0) return 0;
    if (buffer[1] > 1U || buffer[3] > 1U) return 0;
    command->command = VEHICLE_CMD_SET_WHEEL_TARGET;
    command->sequence = 0;
    command->left_target = buffer[1] ? -(short)buffer[2] : (short)buffer[2];
    command->right_target = buffer[3] ? -(short)buffer[4] : (short)buffer[4];
    return 1;
}
