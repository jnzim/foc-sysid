#include "protocol.h"
#include <cstdint>
#include <cstring>

uint16_t crc16_calc(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++)
        {
            crc <<= 1;
            if (crc & 0x10000)
                crc ^= 0x1021;
        }
    }
    return crc;
}
