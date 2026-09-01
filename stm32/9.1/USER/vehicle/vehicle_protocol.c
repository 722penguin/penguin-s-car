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
    if (index == 1 && byte != VEHICLE_SOF1) { index = 0; return 0; }
    buffer[index++] = byte;
    if (index < VEHICLE_FRAME_SIZE) return 0;
    index = 0;
    if (buffer[9] != VEHICLE_EOF0 || buffer[10] != VEHICLE_EOF1) return 0;
    if (VehicleProtocol_Xor(&buffer[2], 6) != buffer[8]) return 0;
    command->command = buffer[2];
    command->sequence = buffer[3];
    command->left_target = (short)((u16)buffer[4] | ((u16)buffer[5] << 8));
    command->right_target = (short)((u16)buffer[6] | ((u16)buffer[7] << 8));
    return 1;
}
