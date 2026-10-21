# foc-sysid

Raspberry Pi side of a bare metal STM32F411 FOC servo: SPI telemetry capture, move commands, and the frequency response analysis behind every loop design.

- Firmware: [jnzim/stm32-servo-drive](https://github.com/jnzim/stm32-servo-drive)
- Write-up with plots and video: https://jnzim.github.io/foc-servo/

## Results

| Loop     | Rate   | Bandwidth (−3 dB) | Phase margin |
|----------|--------|-------------------|--------------|
| Current  | 20 kHz | 767 Hz            | 72.7°        |
| Velocity | 5 kHz  | 171 Hz            | 67.0°        |
| Position | 1 kHz  | 123.8 Hz          | 61.2°        |

A 10 mm trapezoidal move at 10 mm/s tracks to ±0.73 µm (3 encoder counts) at cruise, with friction and velocity feedforward.

## Two programs, one per firmware mode

The Pi program has to match the firmware's `RUN_MODE` (`Include/config.h` in the firmware repo). The two modes send different 32-byte frame formats, so the wrong program decodes garbage.

| Firmware `RUN_MODE`    | Pi program   | What it does |
|------------------------|--------------|--------------|
| `RUN_MODE_SYSID`       | `drive`      | Starts the identification test the firmware was built with, captures `SysIdSample` frames at about 5 kHz, writes `drive_data/latest/sysid_log.csv`, and runs the matching analysis script. |
| `RUN_MODE_CLOSED_LOOP` | `move_telem` | Sends the moves in `move_cmd.txt` as `MoveCmd` frames, captures `TelemetryFrame` telemetry for each move, and writes a CSV and a plot per move. |

Both programs pulse GPIO3 (to STM32 PC3) to release the firmware from its boot wait, so reset the STM32 before each run.

`move_cmd.txt` holds one move per line: `target_mm vel_mm_s accel_mm_s2`, with the target relative to where the stage is.

## Identification tests

The excitation is generated on the STM32, and every frame carries the test ID, so `drive` picks the script itself. IDs are fixed; gaps are tests that were removed.

| ID | Test | Script | Result |
|----|------|--------|--------|
| 0  | Open loop current chirp on v<sub>q</sub> | `bode_plot.py` | R, L |
| 3  | Closed current loop chirp | `closed_current_bode_plot.py` | Current loop crossover and margin |
| 14 | Closed velocity loop chirp, P only | `closed_vel_bode_plot.py` | Velocity plant K, τ |
| 6  | Closed velocity loop chirp, deployed PI | `closed_vel_bode_plot.py` | Velocity loop crossover and margin |
| 8  | Closed position loop chirp | `closed_pos_bode_plot.py` | Position loop crossover and margin |
| 12 | Friction sweep (constant speed holds) | none yet | Coulomb and viscous friction |

Other scripts in `py-script/`:

| Script | Use |
|--------|-----|
| `plot_move.py` | Plot one `move_telem` capture (run by `move_telem`) |
| `analyze_moves.py` | Following error and current model fits across move captures |
| `vel_loop_olt.py` | Velocity loop open loop L = H/(1−H) from a test 6 capture |
| `plot_observer_vs_boxcar.py` | Velocity observer against the encoder boxcar, from a test 6 capture |
| `check_sample_window.py` | Offline check of where each ADC sample falls in its phase's low side window |
| `export_*.py` | Export measured data as JSON for the write-up's charts |

## Hardware

- Raspberry Pi 5, SPI0 at 4 MHz, mode 1
- GPIO3 to STM32 PC3: start signal (active low)
- STM32F411RE Nucleo, TI DRV8353RS, Kollmorgen AKM11E, ball screw stage (2 mm lead)

## Build

```bash
mkdir build && cd build
cmake ..
make -j4
./drive        # or ./move_telem
```

`.vscode/tasks.json` has a build-and-run task for each program.
