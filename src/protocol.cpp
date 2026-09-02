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
            // Test the outgoing bit BEFORE shifting, not after -- see
            // drive repo src/protocol.c for the full explanation. Verified:
            // crc16_calc("123456789", 9) == 0x29B1 (standard CCITT-FALSE).
            if (crc & 0x8000)
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            else
                crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}
