// protocol.h — shared SPI2 protocol definitions
// Used by both STM32 firmware (C) and Raspberry Pi application (C++)
// Keep this file pure C — no C++ types, no includes beyond stdint.h

#pragma once
#include <stdint.h>

// =============================================================================
// Opcodes — byte 0 of every 24-byte SPI transaction
// =============================================================================
#define SPI2_OP_NOP        0x00u  // idle / pad
#define SPI2_OP_BLOCK_HDR  0x03u  // start of trajectory refill block
#define SPI2_OP_DATA       0x04u  // single trajectory sample
#define SPI2_OP_READY_ACK  0x05u  // Pi acknowledges PC13 READY signal
#define SPI2_OP_TELEM_REQ  0x06u  // Pi requests telemetry frame

// =============================================================================
// Transaction size — both sides must agree, every transfer is exactly this
// =============================================================================
#define SPI2_TRANSACTION_BYTES  24u

// =============================================================================
// TelemetryFrame — 24 bytes, STM sends this on MISO every transaction
// __attribute__((packed)) — no padding, fields laid out exactly as written
// Pi casts raw rx bytes directly to this struct — byte order must match
// All multi-byte fields are little-endian (STM32 and Pi 4/5 are both little-endian)
// =============================================================================
typedef struct __attribute__((packed)) {
    int32_t  pos_cmd;       // position command, encoder counts
    int32_t  pos_fbk;       // position feedback from TIM5, encoder counts
    int16_t  vel_cmd;       // velocity command, counts/sec
    int16_t  vel_fbk;       // dx/dt computed in velocity loop, counts/sec
    uint32_t timestamp_ms;  // STM millisecond counter — use this for Bode plot time axis
    uint8_t  drive_state;   // DriveState enum value
    uint8_t  fault_flags;   // fault bitmask
    uint8_t  _pad[6];       // reserved, always 0x00, pads to 24 bytes
} TelemetryFrame;

// =============================================================================
// Drive states — values for TelemetryFrame.drive_state
// =============================================================================
#define DRIVE_IDLE     0x00u  // powered, PWM disabled, waiting for enable
#define DRIVE_ALIGN    0x01u  // executing rotor alignment pulse
#define DRIVE_ENABLED  0x02u  // FOC active, following trajectory
#define DRIVE_FAULT    0x03u  // fault latched, PWM disabled
#define DRIVE_ESTOP    0x04u  // e-stop, controlled decel to zero

// =============================================================================
// Fault flags — bitmask in TelemetryFrame.fault_flags
// =============================================================================
#define FAULT_OCP      (1u << 0)  // overcurrent protection — DRV8353RS tripped
#define FAULT_OVP      (1u << 1)  // overvoltage protection
#define FAULT_UVP      (1u << 2)  // undervoltage protection
#define FAULT_OVERTEMP (1u << 3)  // motor thermistor over temperature
#define FAULT_ENC      (1u << 4)  // encoder loss or TIM5 error
#define FAULT_ALIGN    (1u << 5)  // startup rotor alignment failed
#define FAULT_CRC      (1u << 6)  // SPI2 receive CRC error on command path
#define FAULT_DRV      (1u << 7)  // DRV8353RS nFAULT pin asserted

// =============================================================================
// DATA packet layout — 24 bytes total
// [0]     opcode     SPI2_OP_DATA (0x04)
// [1-4]   int32_t    pos_cmd, little-endian encoder counts
// [5-8]   int32_t    vel_ff, little-endian counts/sec
// [9]     uint8_t    CRC8 XOR over bytes 0-8
// [10-23] uint8_t    pad 0x00
// =============================================================================

// =============================================================================
// BLOCK_HDR packet layout — 24 bytes total
// [0]     opcode     SPI2_OP_BLOCK_HDR (0x03)
// [1]     uint8_t    sample count high byte
// [2]     uint8_t    sample count low byte
// [3]     uint8_t    CRC8 XOR over bytes 0-2
// [4-23]  uint8_t    pad 0x00
// =============================================================================