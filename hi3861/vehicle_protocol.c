#include "vehicle_protocol.h"

uint8_t VehicleProtocol_Xor(const uint8_t *data, uint8_t length)
{
    uint8_t value = 0;
    while (length-- > 0) value ^= *data++;
    return value;
}

uint8_t VehicleProtocol_PackCommand(const VehicleCommand *command, uint8_t *frame)
{
    int16_t left = command->left_target;
    int16_t right = command->right_target;

    frame[0] = VEHICLE_SOF0;
    frame[1] = (left < 0) ? 1U : 0U;
    frame[2] = (uint8_t)((left < 0) ? -left : left);
    frame[3] = (right < 0) ? 1U : 0U;
    frame[4] = (uint8_t)((right < 0) ? -right : right);
    frame[5] = VEHICLE_EOF0;
    return VEHICLE_FRAME_SIZE;
}
