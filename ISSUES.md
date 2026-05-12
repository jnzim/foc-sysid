# Known Issues

## Telemetry / Logging

- **telem cmd x-axis offset** — Pi starts logging slightly late, t0 alignment is approximate. First ~100 samples consumed before logging begins.
- **vel_fbk units** — logged in counts/sec but STM velocity loop runs in rad/s internally. Inconsistent for analysis. Standardize before hardware bring-up.
- **vel_fbk resolution** — int16_t limits velocity feedback range. May need scaling factor for high speed moves.

## Plot

- **Feedback ends before profile** — Pi stops logging when samples_consumed >= profile.size(). Plant may still be coasting. Not a bug but worth noting.
- **telem cmd leads profile cmd** — timestamp offset between Pi clock and STM tick_ms causes apparent lead. Cosmetic only.

## Protocol

- **No move timeout** — if STM stops consuming samples (fault, reset), Pi loops forever. Need watchdog timeout on samples_consumed.
- **No fault frame detection** — Pi never checks drive_state or fault_flags in TelemetryFrame. Should abort move on DRIVE_FAULT.

## Hardware (not started)

- **Pi 5 not tested** — SPI timing and spidev behavior may differ from Pi 4. Verify comms after swap.
- **SPI speed** — running at 1MHz. May need adjustment for Pi 5.

## Build

- **Hardcoded paths** — plotprof.py path hardcoded in system() call in main.cpp. Should be relative or configurable.
- **sudo required** — spidev requires sudo. Should add user to spi group instead.
