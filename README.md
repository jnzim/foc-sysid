# servo-trajectory-streamer

Raspberry Pi side of a custom servo drive project. Generates trapezoidal motion profiles, streams trajectory samples to an STM32F411RE over SPI, collects telemetry, and plots results.

Companion firmware: [stm32-servo-drive](https://github.com/jnzim/stm32-servo-drive)

![Position tracking](docs/tracking.png)

## Architecture

```
Pi 5 (C++) ──SPI 1 MHz──> STM32F411 ring buffer ──ISR──> servo loops ──PWM──> motor
           <──READY (GPIO)──                <──32-byte telemetry──
```

## What it does

- Computes trapezoidal velocity profiles (accel / cruise / decel) from mm inputs
- Converts mm to encoder counts at the boundary — all internal math in counts
- Streams 8-byte samples (int32 position + int32 velocity) to the STM32 over SPI at 1 kHz — proven at 5007 consecutive packets, 0 errors
- Fills a 4096-sample ring buffer on the STM32, refills in 2048-sample blocks when READY asserts
- Collects 32-byte telemetry frames back: position command, position feedback, velocity feedback, position error, q-axis current, q-axis voltage
- Detects move complete via `samples_consumed` — no polling timeout
- Logs `profile.csv` and `telem.csv` after each move
- Auto-plots position tracking, velocity tracking, position error, current, and voltage via matplotlib

## Hardware

- Raspberry Pi 5
- STM32F411RE Nucleo-64 — bare metal, no HAL, no RTOS
- SPI0 at 1 MHz, 25 µs inter-packet delay, manual CS via GPIO
- PC13 READY signal from the STM32 — active low, triggers refill

## Protocol

- Block header (`0x03`) — starts trajectory block, sends sample count
- Data packet (`0x04`) — 8-byte sample + XOR checksum, padded to 32 bytes for deterministic DMA buffer alignment
- READY ACK (`0x05`) — Pi acknowledges PC13 assertion
- Telemetry request (`0x06`) — STM32 replies with 32-byte TelemetryFrame on MISO

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
- Ring buffer + block refill: proven
- Telemetry: 32-byte frame live; pos / vel / pos_err / i_q / v_q all logging
- Velocity loop: active, plant responding
- Position loop: next
- Motor integration: encoder bring-up complete (Kollmorgen AKM11E, 8192-count RS-422 differential via AM26LS32 + hardware quadrature decode); FOC current loop next

## Project goal

Full-stack motion control from scratch: trajectory generation on Linux, a custom SPI streaming protocol, and a bare-metal FOC servo drive — every layer written and debugged at register level, no vendor frameworks.