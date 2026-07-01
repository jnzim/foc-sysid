# foc-sysid

Raspberry Pi data capture and frequency-domain analysis for a bare-metal STM32 FOC servo drive.

Companion firmware: [stm32-servo-drive](https://github.com/jnzim/stm32-servo-drive)

![Current Plant Bode](docs/bode_plot.png)

## What it does

- Captures 32-byte telemetry frames from STM32F411 over SPI at ~10kHz (Pi 5 SPI limit)
- Triggers STM32 sysid sequence via GPIO handshake
- Logs timestamped CSV: encoder position, d/q currents, voltage commands, chirp frequency
- Computes current loop plant Bode plot via Welch CSD estimation
- Extracts R, L, fc from measured frequency response
- Overlays confirmed plant model H(s) = 1/(Ls + R) with phase margin analysis

## Current results — Kollmorgen AKM11E

| Parameter | Measured | Datasheet (line-to-line) | Method |
|-----------|----------|--------------------------|--------|
| R | 3.55 Ω | 3.10 Ω | Bode DC magnitude |
| L | 2.47 mH | 2.04 mH | Bode -3dB, fc = 229 Hz |
| fc | 229 Hz | 241 Hz | Bode -3dB from flat |

> Measured R and L are slightly higher than datasheet — consistent with additional series resistance and inductance from a long motor cable in the test setup.

## Hardware

- Raspberry Pi 5
- STM32F411RE Nucleo-64 — bare metal, no HAL
- Texas Instruments DRV8353RS-EVM gate driver + 7mΩ shunt resistors, CSA gain 10 V/V
- Kollmorgen AKM11E BLDC motor, 3 pole pairs, 8192-count RS-422 encoder
- SPI0 at 4 MHz — telemetry at ~10kHz
- GPIO3 → STM PC3 active-low trigger

## Build

```bash
mkdir build && cd build
cmake ..
make -j4
./drive
```

## Analysis

```bash
python3 py-script/bode_plot.py drive_data/sysid_log.csv
```

## Roadmap

- [x] Current loop plant sysid — R, L, fc confirmed
- [x] Bode plot with Welch CSD, coherence, phase margin analysis
- [ ] Current loop PI closure
- [ ] Closed loop step response
- [ ] Velocity loop plant sysid — J, B, Kt
- [ ] Velocity loop closure