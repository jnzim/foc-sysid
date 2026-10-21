#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* =============================================================================
 * Protocol constants
 * =============================================================================
 */
#define SPI2_TRANSACTION_BYTES  32u

/* Minimum CS-high time between transactions. The STM re-arms SPI2 on every
 * CS rising edge (drive spi.c); that handler can wait behind the 20 kHz
 * control ISR, so the next transaction must not start sooner than this.
 * Sized from sysid, the heaviest mode -- see the drive protocol.h. */
#define SPI2_MIN_CS_GAP_US      120u

/* Opcodes (wire values shared with the STM; never renumber) */
#define SPI2_OP_MOVE_CMD        0x07  /* single-command move -- see MoveCmd below */

/* Drive state machine values (wire values; 1 and 2 were OPEN_LOOP/ALIGN) */
#define DRIVE_IDLE              0x00u
#define DRIVE_SERVO_ON          0x03u
#define DRIVE_FAULT             0x04u

/* Fault flags */
#define FAULT_FOLLOWING_ERROR   0x08  /* |pos_cmd - pos_fbk| > FOLLOWING_ERROR_LIMIT (drive config.h) */

/* =============================================================================
 * TelemetryFrame — 32 bytes
 * STM → Pi status frame
 * =============================================================================
 */
typedef struct __attribute__((packed)) {
    int32_t  pos_cmd;           /* counts */
    int32_t  pos_fbk;           /* counts */
    int32_t  vel_cmd;           /* counts/s */
    int32_t  vel_fbk;           /* counts/s */
    uint32_t timestamp_ms;
    uint8_t  drive_state;       /* DRIVE_IDLE/DRIVE_SERVO_ON/etc above */
    uint8_t  fault_flags;
    uint16_t samples_consumed;  /* 1 kHz ticks into the move; wraps at 65 s */
    uint8_t  move_seq;          /* seq of the last MoveCmd accepted (the ack) */
    uint8_t  spare;
    int16_t  iq_cmd;            /* mA */
    int16_t  i_q_fbk;           /* mA */
    uint16_t crc16;             /* covers bytes 0..29 -- TELEM_CRC_LEN below */
} TelemetryFrame;

/* =============================================================================
 * MoveCmd — 32 bytes, one per move (SPI2_OP_MOVE_CMD)
 *
 * Pi -> STM: one command per move. The STM generates the profile itself
 * (trap_gen.c or minjerk_gen.c, per TRAJ_PROFILE) from these three numbers.
 *
 * seq is the handshake: the STM runs a MoveCmd only if its seq differs from
 * the last one it accepted, and echoes that seq in TelemetryFrame.move_seq.
 * The Pi picks seq = move_seq + 1 (skipping 0) and resends until the echo
 * matches, so a lost frame is retried and a repeat is never run twice.
 *
 * Structure:
 *   [0]      opcode (0x07 = MOVE_CMD)
 *   [1]      seq (never 0)
 *   [2..5]   target_pos (int32, counts, RELATIVE to wherever the stage
 *            actually is when the move starts -- a delta, not an absolute
 *            position. Sending the same MoveCmd twice moves twice.)
 *   [6..9]   vel_max    (int32, counts/s, magnitude)
 *   [10..13] accel_max  (int32, counts/s^2, magnitude)
 *   [14..15] crc16 (covers bytes 0..13)
 *   [16..31] padding (zeros)
 * =============================================================================
 */
typedef struct __attribute__((packed)) {
    uint8_t  opcode;       /* 1 */
    uint8_t  seq;          /* 1 */
    int32_t  target_pos;   /* 4 -- delta from current position, see above */
    int32_t  vel_max;      /* 4 */
    int32_t  accel_max;    /* 4 */
    uint16_t crc16;        /* 2 */
    uint8_t  padding[16];  /* 16 — total 32 */
} MoveCmd;

/* =============================================================================
 * Size assertions — C11/C++ compatible
 * =============================================================================
 */
#ifdef __cplusplus
static_assert(sizeof(TelemetryFrame) == 32, "TelemetryFrame must be exactly 32 bytes");
static_assert(sizeof(MoveCmd) == 32, "MoveCmd must be exactly 32 bytes");
#else
_Static_assert(sizeof(TelemetryFrame) == 32, "TelemetryFrame must be exactly 32 bytes");
_Static_assert(sizeof(MoveCmd) == 32, "MoveCmd must be exactly 32 bytes");
#endif

#define TELEM_CRC_LEN     offsetof(TelemetryFrame, crc16)
#define MOVE_CMD_CRC_LEN  offsetof(MoveCmd, crc16)

/* =============================================================================
 * CRC-16-CCITT utilities
 * =============================================================================
 */
uint16_t crc16_calc(const uint8_t* data, size_t len);

#endif