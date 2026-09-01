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
            // See drive repo's src/protocol.c for why this must test the
            // outgoing bit BEFORE shifting: the previous "crc <<= 1; if
            // (crc & 0x10000)" form could never trigger (crc is uint16_t,
            // already truncated by the shift-assign before the check),
            // making this function return 0x0000 for any input >= 2 bytes
            // regardless of content -- verified on the host.
            if (crc & 0x8000)
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            else
                crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

bool crc16_valid(const TrajSlot* s, size_t crc_len)
{
    uint16_t calc = crc16_calc((const uint8_t*)s, crc_len);
    return (calc == s->crc16);
}
