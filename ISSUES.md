# servo-trajectory-streamer
Raspberry Pi side of a custom servo drive project. Generates trapezoidal motion profiles, streams trajectory samples to an STM32F446RE over SPI, collects telemetry, and plots results.

Companion firmware: [stm32-servo-drive](https://github.com/jnzim/stm32-servo-drive)

## What it does
- Computes trapezoidal velocity profiles (accel / cruise / decel) from mm inputs
- Converts mm to encoder counts at the boundary — all internal math in counts
- Streams 8-byte samples (int32 position + int32 velocity) to STM32 over SPI at 1 kHz
- Fills a 4096-sample ring buffer on the STM, refills in 2048-sample blocks when READY asserts
- Collects 32-byte telemetry frames back from the STM: position command, position feedback, velocity feedback, position error, q-axis current, q-axis voltage
- Detects move complete via `samples_consumed` — no polling timeout
- Logs `profile.csv` and `telem.csv` after each move
- Auto-plots position tracking, velocity tracking, position error, current, and voltage via matplotlib

## Hardware
- Raspberry Pi 4 (Pi 5 target)
- STM32F446RE Nucleo-64 — bare metal, no HAL
- SPI0 at 1 MHz, 25µs inter-packet delay, manual CS via GPIO
- PC13 READY signal from STM — active low, triggers refill

## Protocol
- Block header (0x03) — starts trajectory block, sends sample count
- Data packet (0x04) — 8-byte sample + CRC8 XOR, padded to 32 bytes
- Telemetry request (0x06) — STM replies with 32-byte TelemetryFrame on MISO
- READY ACK (0x05) — Pi acknowledges PC13 assertion

## Build
```bash
mkdir build && cd build
cmake ..
cmake --build . -j4
./drive
```

## Plot
```bash
python3 py-script/plotprof.py docs/profile.csv docs/telem.csv
```

## Status
- SPI streaming: proven — 5007 packets, 0 errors
- Ring buffer + refill: proven
- Telemetry: 32-byte frame live, pos/vel/pos_err/i_q/v_q all logging
- Velocity loop: active, plant responding
- Position loop: next
- Real motor: not yet

## Project goal
Close the FOC current loop from scratch on bare metal STM32 — full stack controls and embedded competency demonstration targeting semiconductor capital equipment (KLA, ASML, Aerotech).