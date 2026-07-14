# foc-sysid

Raspberry Pi telemetry capture and post-processing tools for a bare-metal STM32 FOC servo drive.

Companion firmware: [stm32-servo-drive](https://github.com/jnzim/stm32-servo-drive)

![Current Plant Bode](docs/bode_plot.png)

## Architecture

The excitation profiles are generated directly on the STM32.

Both the chirp used for plant identification and the closed-loop current step sequence are simple enough to calculate in real time inside the control firmware. Keeping profile generation on the STM32 avoids sending commands across SPI during the test and allows the telemetry return rate to increase from 1 kHz to approximately 10 kHz.

During a capture:

1. The Raspberry Pi triggers the test through a GPIO handshake.
2. The STM32 generates the chirp or step profile in real time.
3. The STM32 runs the control loop and streams telemetry back over SPI.
4. The Raspberry Pi continuously reads the SPI telemetry and appends each frame to an in-memory vector.
5. After the test finishes, the Raspberry Pi writes the complete capture to CSV.
6. Python scripts post-process the CSV data and generate the system-identification and step-response plots.

The data path during the test is therefore primarily:

```text
STM32 profile generation and control
              |
              v
      SPI telemetry stream
              |
              v
 Raspberry Pi memory buffer
              |
              v
       CSV after capture
              |
              v
      Python post-processing
```

## What it does

- Triggers STM32 system-identification captures through a GPIO handshake
- Reads 32-byte telemetry frames from the STM32 over SPI at approximately 10 kHz
- Buffers the complete capture in memory before writing it to disk
- Writes timestamped CSV data containing encoder position, d/q currents, voltage commands, and excitation state
- Generates chirp and closed-loop step profiles directly on the STM32
- Computes the current-plant Bode plot using Welch CSD estimation
- Extracts R, L, and corner frequency from the measured frequency response
- Overlays the confirmed plant model `H(s) = 1 / (Ls + R)`
- Plots closed-loop q-axis current tracking, PI output, and phase currents

## Current results — Kollmorgen AKM11E

| Parameter | Measured | Datasheet (line-to-line) | Method |
|-----------|----------|--------------------------|--------|
| R | 3.55 Ω | 3.10 Ω | Bode DC magnitude |
| L | 2.47 mH | 2.04 mH | Bode -3 dB, `fc = 229 Hz` |
| fc | 229 Hz | 241 Hz | Bode -3 dB from flat response |

> Measured R and L are slightly higher than the datasheet values, consistent with additional series resistance and inductance from the long motor cable used in the test setup.

## Closed-loop current response

The current-loop PI gains were calculated from the measured first-order motor model:

\[
G(s)=\frac{1}{Ls+R}
\]

with:

- \(R = 3.55\ \Omega\)
- \(L = 2.47\ \text{mH}\)

The controller was defined as:

\[
C(s)=K_p+\frac{K_i}{s}
\]

The PI zero was placed on the measured electrical pole:

\[
\frac{K_i}{K_p}=\frac{R}{L}
\]

This cancels the plant pole and reduces the open-loop response to an integrator. The desired current-loop bandwidth was then set to 500 Hz:

\[
\omega_c = 2\pi(500)=3142\ \text{rad/s}
\]

The resulting gains were:

\[
K_p=L\omega_c
     =(2.47\times10^{-3})(3142)
     \approx 7.76
\]

\[
K_i=R\omega_c
     =(3.55)(3142)
     \approx 11153
\]

These gains were implemented on the STM32 in the 20 kHz current-control loop.

The STM32 generates the positive and negative current steps internally while returning telemetry to the Raspberry Pi at approximately 10 kHz.

![Closed-Loop Current Step Response](docs/step_response.png)

The measured q-axis current tracks the commanded ±0.5 A steps with little steady-state error. The rotor was mechanically locked during this test.

## Hardware

- Raspberry Pi 5
- STM32F411RE Nucleo-64 — bare metal, no HAL
- Texas Instruments DRV8353RS-EVM gate driver
- 7 mΩ low-side shunt resistors with CSA gain of 40 V/V
- Kollmorgen AKM11E BLDC motor
- 3 pole pairs
- 8192-count RS-422 encoder
- SPI0 at 4 MHz
- Telemetry capture at approximately 10 kHz
- GPIO3 to STM32 PC3 active-low test trigger

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

- [x] STM32-generated current-loop chirp excitation
- [x] Raspberry Pi SPI telemetry capture at approximately 10 kHz
- [x] Buffered capture followed by CSV export
- [x] Current-loop plant sysid — R, L, and fc confirmed
- [x] Bode plot with Welch CSD and coherence analysis
- [x] Current-loop PI closure
- [x] STM32-generated closed-loop current step sequence
- [x] Closed-loop q-axis current step response
- [ ] Velocity-loop plant sysid — J, B, and Kt
- [ ] Velocity-loop closure
- [ ] Position-loop closure
