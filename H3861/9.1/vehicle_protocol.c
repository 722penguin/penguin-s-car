#include "vehicle_protocol.h"

uint8_t VehicleProtocol_Xor(const uint8_t *data, uint8_t length)
{
    uint8_t value = 0;
    while (length-- > 0) value ^= *data++;
    return value;
}

uint8_t VehicleProtocol_PackCommand(const VehicleCommand *command, uint8_t *frame)
{
    frame[0] = VEHICLE_SOF0;
    frame[1] = VEHICLE_SOF1;
    frame[2] = command->command;
    frame[3] = command->sequence;
    frame[4] = (uint8_t)(command->left_target & 0xFF);
    frame[5] = (uint8_t)((uint16_t)command->left_target >> 8);
    frame[6] = (uint8_t)(command->right_target & 0xFF);
    frame[7] = (uint8_t)((uint16_t)command->right_target >> 8);
    frame[8] = VehicleProtocol_Xor(&frame[2], 6);
    frame[9] = VEHICLE_EOF0;
    frame[10] = VEHICLE_EOF1;
    return VEHICLE_FRAME_SIZE;
}
