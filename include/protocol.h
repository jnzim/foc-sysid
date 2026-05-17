// protocol.h — shared SPI2 protocol definitions
// Used by both STM32 firmware (C) and Raspberry Pi application (C++)
// Keep this file pure C — no C++ types, no includes beyond stdint.h

#pragma once
#include <stdint.h>

#define ENCODER_CPR         2048u                   // counts per revolution (raw)
#define ENCODER_COUNTS_REV  (ENCODER_CPR * 4u)      // 4x quadrature decode = 8192
#define COUNTS_PER_RAD      (ENCODER_COUNTS_REV / 6.2832f)

// =============================================================================
// Opcodes — byte 0 of every SPI transaction
// =============================================================================
#define SPI2_OP_NOP        0x00u
#define SPI2_OP_BLOCK_HDR  0x03u
#define SPI2_OP_DATA       0x04u
#define SPI2_OP_READY_ACK  0x05u
#define SPI2_OP_TELEM_REQ  0x06u

// =============================================================================
// Transaction size — both sides must agree
// =============================================================================
#define SPI2_TRANSACTION_BYTES  32u

// =============================================================================
// TelemetryFrame — 32 bytes
// STM sends this on MISO every transaction
// Pi casts raw rx bytes directly to this struct
// All multi-byte fields little-endian (STM32 and Pi 4/5 are both LE)
// =============================================================================
typedef struct __attribute__((packed)) {
    int32_t  pos_cmd;           //  4  — ring buffer setpoint (counts)
    int32_t  pos_fbk;           //  4  — plant position (counts)
    int16_t  vel_cmd;           //  2  — velocity setpoint (counts/s)
    int16_t  vel_fbk;           //  2  — plant velocity (counts/s)
    uint32_t timestamp_ms;      //  4  — SysTick ms counter
    uint8_t  drive_state;       //  1  — DRIVE_* state
    uint8_t  fault_flags;       //  1  — FAULT_* bitmask
    uint32_t samples_consumed;  //  4  — ring pops, Pi uses for move complete
    int16_t  pos_err;           //  2  — pos_cmd - pos_fbk (counts)
    int16_t  i_q_fbk;           //  2  — plant.i_q * 1000 (mA resolution)
    float    v_q_cmd;           //  4  — raw voltage into plant (V)
    uint8_t  _pad[2];           //  2  — reserved, always 0x00
} TelemetryFrame;               // 32 bytes total

// =============================================================================
// TrajSample — 8 bytes, Pi → STM ring buffer
// =============================================================================
typedef struct __attribute__((packed)) {
    int32_t pos_cmd;  // encoder counts, little-endian
    int32_t vel_cmd;  // counts/sec, little-endian
} TrajSample;

// =============================================================================
// Drive states
// =============================================================================
#define DRIVE_IDLE     0x00u
#define DRIVE_ALIGN    0x01u
#define DRIVE_ENABLED  0x02u
#define DRIVE_FAULT    0x03u
#define DRIVE_ESTOP    0x04u

// =============================================================================
// Fault flags
// =============================================================================
#define FAULT_OCP      (1u << 0)
#define FAULT_OVP      (1u << 1)
#define FAULT_UVP      (1u << 2)
#define FAULT_OVERTEMP (1u << 3)
#define FAULT_ENC      (1u << 4)
#define FAULT_ALIGN    (1u << 5)
#define FAULT_CRC      (1u << 6)
#define FAULT_DRV      (1u << 7)

// =============================================================================
// DATA packet layout — 32 bytes total
// [0]     opcode     SPI2_OP_DATA (0x04)
// [1-4]   int32_t    pos_cmd, little-endian encoder counts
// [5-8]   int32_t    vel_ff, little-endian counts/sec
// [9]     uint8_t    CRC8 XOR over bytes 0-8
// [10-31] uint8_t    pad 0x00
// =============================================================================

// =============================================================================
// BLOCK_HDR packet layout — 32 bytes total
// [0]     opcode     SPI2_OP_BLOCK_HDR (0x03)
// [1]     uint8_t    sample count high byte
// [2]     uint8_t    sample count low byte
// [3]     uint8_t    CRC8 XOR over bytes 0-2
// [4-31]  uint8_t    pad 0x00
// =============================================================================