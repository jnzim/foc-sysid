# foc-sysid

Raspberry Pi data capture and frequency-domain analysis for a bare-metal STM32 FOC servo drive.

Companion firmware: [stm32-servo-drive](https://github.com/jnzim/stm32-servo-drive)

![Current Plant Bode](docs/bode_plot.png)

## What it does

- Captures 32-byte telemetry frames from STM32F411 over SPI at ~10 kHz
- Triggers the STM32 sysid sequence through a GPIO handshake
- Logs timestamped CSV data including encoder position, d/q currents, voltage commands, and chirp frequency
- Computes the current-loop plant Bode plot using Welch CSD estimation
- Extracts R, L, and corner frequency from the measured frequency response
- Overlays the confirmed plant model `H(s) = 1 / (Ls + R)` with phase-margin analysis
- Captures and plots closed-loop q-axis current step responses

## Current results — Kollmorgen AKM11E

| Parameter | Measured | Datasheet (line-to-line) | Method |
|-----------|----------|--------------------------|--------|
| R | 3.55 Ω | 3.10 Ω | Bode DC magnitude |
| L | 2.47 mH | 2.04 mH | Bode -3 dB, `fc = 229 Hz` |
| fc | 229 Hz | 241 Hz | Bode -3 dB from flat response |

> Measured R and L are slightly higher than the datasheet values, consistent with additional series resistance and inductance from the long motor cable used in the test setup.

## Closed-loop current response

The current-loop PI gains were calculated from the measured first-order RL plant and implemented on the STM32 at 20 kHz.

The plot below shows the commanded and measured q-axis current for positive and negative 0.5 A steps, along with the PI voltage output and measured phase currents.

![Closed-Loop Current Step Response](docs/step_response.png)

The measured q-axis current tracks the command with little steady-state error. The rotor was mechanically locked during this test.

## Hardware

- Raspberry Pi 5
- STM32F411RE Nucleo-64 — bare metal, no HAL
- Texas Instruments DRV8353RS-EVM gate driver
- 7 mΩ low-side shunt resistors with CSA gain of 10 V/V
- Kollmorgen AKM11E BLDC motor
- 3 pole pairs
- 8192-count RS-422 encoder
- SPI0 at 4 MHz with telemetry capture at ~10 kHz
- GPIO3 to STM32 PC3 active-low trigger

## Build

```bash
mkdir build
cd build
cmake ..
make -j4
./drive
```

## Analysis

Generate the measured current-plant Bode plot:

```bash
python3 py-script/bode_plot.py drive_data/sysid_log.csv
```

Generate the closed-loop current step-response plot:

```bash
python3 py-script/step.py drive_data/sysid_log.csv
```

The scripts save:

```text
bode_plot.png
step_response.png
```

Copy the generated images into `docs/` before committing:

```bash
cp bode_plot.png docs/bode_plot.png
cp step_response.png docs/step_response.png
```

## Roadmap

- [x] Current-loop plant sysid — R, L, and fc confirmed
- [x] Bode plot with Welch CSD, coherence, and phase-margin analysis
- [x] Current-loop PI closure
- [x] Closed-loop q-axis current step response
- [ ] Validate orthogonal current measurement and transform conventions
- [ ] Velocity-loop plant sysid — J, B, and Kt
- [ ] Velocity-loop closure
- [ ] Position-loop closure