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
#define CRC16_POLY              0x1021  /* CCITT polynomial */

/* Opcodes — Pi → STM commands and STM → Pi responses */
#define SPI2_OP_NOP             0x00
#define SPI2_OP_BLOCK_HDR       0x01
#define SPI2_OP_DATA            0x02
#define SPI2_OP_READY_ACK       0x03
#define SPI2_OP_TELEM_REQ       0x04
#define SPI2_OP_OPEN_LOOP       0x05
#define SPI2_OP_STOP            0x06
#define SPI2_OP_MOVE_CMD        0x07  /* M2: single-command move -- see MoveCmd below */

/* Drive state machine values */
#define DRIVE_IDLE              0x00u
#define DRIVE_OPEN_LOOP         0x01u
#define DRIVE_ALIGN             0x02u
#define DRIVE_SERVO_ON          0x03u
#define DRIVE_FAULT             0x04u

/* Trajectory frame fault flags */
#define FAULT_TRAJ_CRC          0x01
#define FAULT_TRAJ_DROP         0x02
#define FAULT_TRAJ_OVF          0x04

/* =============================================================================
 * TelemetryFrame — 32 bytes
 * STM → Pi status frame
 *
 * NOTE: this file intentionally does NOT define SysIdSample -- that's a
 * separate, decode-only convention main.cpp defines locally with its own
 * per-test-repurposable field names (enc_hi/enc_lo/sysid_f/iq_cmd_mA/etc,
 * see main.cpp's own `struct SysIdSample`). Defining it here too would
 * collide with that local definition (confirmed: this exact collision was
 * hit and reverted while adding BlockHdr/TelemetryFrame below -- don't
 * reintroduce it).
 * =============================================================================
 */
typedef struct __attribute__((packed)) {
    int32_t  pos_cmd;
    int32_t  pos_fbk;
    int32_t  vel_cmd;
    int32_t  vel_fbk;
    uint32_t timestamp_ms;
    uint8_t  drive_state;
    uint8_t  fault_flags;
    uint32_t samples_consumed;
    int16_t  iq_cmd;
    int16_t  i_q_fbk;
    uint16_t crc16;    /* covers bytes 0..(offsetof-1) -- TELEM_CRC_LEN below.
                        * Was v_q_cmd -- dropped to make room. This was the one
                        * 32-byte frame in the protocol with no self-check at
                        * all (TrajSlot already has one); cheapest time to fix
                        * it is before anything populates it. */
} TelemetryFrame;

/* =============================================================================
 * TrajSlot — 32 bytes (fixed-size, DMA-aligned)
 *
 * Pi generates trajectory at 1 kHz, sends 32-byte slots to STM.
 * Ring buffer: 4096 bytes / 32 bytes per slot = 128 slots exact.
 * DMA wrap always lands on slot boundary — no partial packets.
 *
 * Structure:
 *   [0]      opcode (0x02 = DATA, never scanned in payload)
 *   [1]      seq (sequence number, detects drops)
 *   [2..5]   pos_cmd (int32, counts)
 *   [6..9]   vel_cmd (int32, counts/s feedforward)
 *   [10..13] reserved (future expansion)
 *   [14..15] crc16 (covers bytes 0..13)
 *   [16..31] padding (zeros)
 *
 * Parser: fixed-offset memcpy, validate opcode + CRC.
 * If corrupt: reuse last valid (pos_cmd, vel_cmd) for current DT.
 * Never scan payload for frame delimiters — eliminates 0x01/0x02/0x03 corruption.
 *
 * M2 note: this struct is retained (real, verified layout) but is no
 * longer the primary command path -- see MoveCmd below.
 * =============================================================================
 */
typedef struct __attribute__((packed)) {
    uint8_t  opcode;      /* 1 */
    uint8_t  seq;         /* 1 */
    int32_t  pos_cmd;     /* 4 */
    int32_t  vel_cmd;     /* 4 */
    uint32_t reserved;    /* 4 */
    uint16_t crc16;       /* 2 */
    uint8_t  padding[16]; /* 16 — total 32 */
} TrajSlot;

/* =============================================================================
 * BlockHdr — 32 bytes, sent once per move (SPI2_OP_BLOCK_HDR) before the DATA
 * stream. Layout confirmed against the actual working trajectory-streamer
 * prototype (jz-dev branch, src/main.cpp) -- not invented: sample_count is a
 * 16-bit field at offset 1-2 (matches the STM side's existing
 * uint16_t expected_samples), not the 32-bit count an initial draft assumed.
 *
 * Structure:
 *   [0]      opcode (0x01 = BLOCK_HDR)
 *   [1..2]   sample_count (uint16, total TrajSlots in this move)
 *   [3..13]  reserved (zero)
 *   [14..15] crc16 (covers bytes 0..13, same TRAJ_CRC_LEN convention as TrajSlot)
 *   [16..31] padding (zeros)
 *
 * M2 note: retained (real, verified layout) but no longer the primary
 * command path -- see MoveCmd below.
 * =============================================================================
 */
typedef struct __attribute__((packed)) {
    uint8_t  opcode;       /* 1 */
    uint16_t sample_count; /* 2 */
    uint8_t  reserved[11]; /* 11 */
    uint16_t crc16;        /* 2 */
    uint8_t  padding[16];  /* 16 — total 32 */
} BlockHdr;

/* =============================================================================
 * MoveCmd — 32 bytes, sent once per move (SPI2_OP_MOVE_CMD)
 *
 * M2 architecture pivot: the STM generates the trapezoidal profile itself
 * (trap_gen.c) from this single command, rather than the Pi streaming a
 * precomputed sample vector through a ring buffer. Given the SPI link's
 * demonstrated corruption under heavy switching, this is a real robustness
 * win, not just simpler code: command traffic drops from a continuous
 * 1kHz stream (thousands of packets per move, each exposed to that
 * corruption) to one small frame. A corrupted MoveCmd just fails its own
 * CRC and never starts a move -- no move is a far safer failure mode than
 * a move built from a corrupted mid-stream sample would have been.
 *
 * Structure:
 *   [0]      opcode (0x07 = MOVE_CMD)
 *   [1]      seq
 *   [2..5]   target_pos (int32, counts)
 *   [6..9]   vel_max (int32, counts/s, magnitude)
 *   [10..13] accel_max (int32, counts/s^2, magnitude)
 *   [14..15] crc16 (covers bytes 0..13, same convention as TrajSlot/BlockHdr)
 *   [16..31] padding (zeros)
 * =============================================================================
 */
typedef struct __attribute__((packed)) {
    uint8_t  opcode;       /* 1 */
    uint8_t  seq;          /* 1 */
    int32_t  target_pos;   /* 4 */
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
static_assert(sizeof(TrajSlot) == 32, "TrajSlot must be exactly 32 bytes");
static_assert(sizeof(BlockHdr) == 32, "BlockHdr must be exactly 32 bytes");
static_assert(sizeof(MoveCmd) == 32, "MoveCmd must be exactly 32 bytes");
#else
_Static_assert(sizeof(TelemetryFrame) == 32, "TelemetryFrame must be exactly 32 bytes");
_Static_assert(sizeof(TrajSlot) == 32, "TrajSlot must be exactly 32 bytes");
_Static_assert(sizeof(BlockHdr) == 32, "BlockHdr must be exactly 32 bytes");
_Static_assert(sizeof(MoveCmd) == 32, "MoveCmd must be exactly 32 bytes");
#endif

#define TRAJ_CRC_LEN      offsetof(TrajSlot, crc16)
#define TELEM_CRC_LEN     offsetof(TelemetryFrame, crc16)
#define BLOCK_HDR_CRC_LEN offsetof(BlockHdr, crc16)
#define MOVE_CMD_CRC_LEN  offsetof(MoveCmd, crc16)

/* =============================================================================
 * CRC-16-CCITT utilities
 * =============================================================================
 */
uint16_t crc16_calc(const uint8_t* data, size_t len);
bool crc16_valid(const TrajSlot* s, size_t crc_len);

#endif
