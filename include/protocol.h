#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

// =============================================================================
// SPI2 transaction size — every frame is exactly 32 bytes
// =============================================================================
#define SPI2_TRANSACTION_BYTES 32u

// =============================================================================
// Opcodes
// =============================================================================
#define SPI2_OP_NOP        0x00
#define SPI2_OP_BLOCK_HDR  0x01
#define SPI2_OP_DATA       0x02
#define SPI2_OP_READY_ACK  0x03
#define SPI2_OP_TELEM_REQ  0x04
#define SPI2_OP_OPEN_LOOP  0x05
#define SPI2_OP_STOP       0x06

// =============================================================================
// TelemetryFrame — 32 bytes
// STM → Pi status frame.
// =============================================================================
typedef struct __attribute__((packed)) {
    int32_t  pos_cmd;           //  4   last command STM consumed
    int32_t  pos_fbk;           //  4   actual encoder position
    int32_t  vel_cmd;           //  4   last vel command
    int32_t  vel_fbk;           //  4   actual velocity
    uint32_t timestamp_ms;      //  4
    uint8_t  drive_state;       //  1
    uint8_t  fault_flags;       //  1
    uint32_t samples_consumed;  //  4
    int16_t  pos_err;           //  2
    int16_t  i_q_fbk;           //  2
    int16_t  v_q_cmd;           //  2   q-axis voltage command
} TelemetryFrame;               // 32 bytes total

// =============================================================================
// TrajSample — 8 bytes
// Pi sends int32 position + int32 velocity.
// STM stores this in the trajectory ring buffer.
// =============================================================================
typedef struct __attribute__((packed)) {
    int32_t pos_cmd;   // 4
    int32_t vel_cmd;   // 4
} TrajSample;          // 8 bytes total

#endif