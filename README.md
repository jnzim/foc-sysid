# servo-trajectory-streamer

Raspberry Pi side of a custom servo drive project. Generates trapezoidal motion profiles, streams trajectory samples to an STM32F446RE over SPI, collects telemetry, and plots results.

Companion firmware: [stm32-servo-drive](https://github.com/jnzim/stm32-servo-drive)

## What it does

- Computes trapezoidal velocity profiles (accel / cruise / decel) from mm inputs
- Converts mm to encoder counts at the boundary — all internal math in counts
- Streams 6-byte samples (int32 position + int16 velocity) to STM32 over SPI at 1 kHz
- Fills a 4096-sample ring buffer on the STM, refills in 2048-sample blocks when READY asserts
- Collects telemetry frames back from the STM (position command, position feedback, velocity feedback, samples consumed)
- Detects move complete via `samples_consumed` — no polling timeout
- Logs `profile.csv` and `telem.csv` after each move
- Auto-plots position and velocity tracking via matplotlib

## Hardware

- Raspberry Pi 4 (Pi 5 target)
- STM32F446RE Nucleo-64 — bare metal, no HAL
- SPI0 at 1 MHz, CE0
- PC13 READY signal from STM — active low, triggers refill

## Protocol

- Block header (0x03) — resets ring buffer, starts move
- Data packet (0x04) — 6-byte sample + CRC8 XOR, padded to 24 bytes
- Telemetry request (0x06) — STM replies with 24-byte TelemetryFrame
- READY ACK (0x05) — Pi acknowledges PC13 assertion

## Build

```bash
mkdir build && cd build
cmake ..
cmake --build . -j4
sudo ./MotionControl
```

## Plot

```bash
python3 py-script/plotprof.py MotionController/profile.csv MotionController/telem.csv
```

## Status

- SPI streaming: proven — 5007 packets, 0 errors
- Ring buffer + refill: proven
- Telemetry: position and velocity feedback live
- Control loop sim: in progress (STM side)
- Real motor: not yet

## Project goal

Close the FOC current loop from scratch on bare metal STM32 — full stack controls and embedded competency demonstration targeting semiconductor capital equipment (KLA, ASML, Aerotech).

