#!/usr/bin/env python3
"""Offline ADC sample-window check.

Recomputes, for every logged tick, where each phase's ADC sample falls relative
to that phase's low-side conduction window.  Uses the same math the firmware
uses: pwm_apply_dq -> inverse Park/Clarke -> volts_to_duty -> CCR.

Sample instants (counts from the counter peak, ARR):
    A = -(OFFSET - 60), B = -(OFFSET - 168), C = -(OFFSET - 276)
with OFFSET = PWM_SAMPLE_OFFSET, and 60/168/276 = the 15-cycle ADC sequence's
hold instants (0.6 / 1.68 / 2.76 us at 100 MHz timer ticks).

Phase X's low side is on while CNT > CCRx, i.e. within +/-(ARR - CCRx) of the
peak.  MARGIN_COUNTS covers DRV dead time + CSA settling.
"""
import csv, math, sys

ARR = 2499
CENTER = ARR // 2
V_BUS = 12.0
DUTY_MIN = ARR * 0.04
DUTY_MAX = ARR * 0.96
MARGIN_COUNTS = 20          # 200 ns: 100 ns DRV dead time + settling allowance
HOLD = (60, 168, 276)       # counts after trigger for A, B, C (15-cycle SMPR)

def volts_to_duty(v):
    n = max(-1.0, min(1.0, v / (V_BUS / 2.0)))
    ccr = n * CENTER + CENTER
    return max(DUTY_MIN, min(DUTY_MAX, ccr))

def phase_volts(vd, vq, theta):
    c, s = math.cos(theta), math.sin(theta)
    va_ = vd * c - vq * s
    vb_ = vd * s + vq * c
    return (va_, -0.5 * va_ + 0.8660254 * vb_, -0.5 * va_ - 0.8660254 * vb_)

def main(path, offset):
    sample_at = [h - offset for h in HOLD]      # counts from peak (+ = after)
    viol = [0, 0, 0]
    worst = [0.0, 0.0, 0.0]
    n = 0
    for r in csv.DictReader(open(path)):
        if r['flags'] in ('0x1234',):
            continue
        vd = int(r['vd_mV']) / 1000.0
        vq = int(r['vq_mV']) / 1000.0
        th = int(r['theta_mrad']) / 1000.0
        n += 1
        for i, v in enumerate(phase_volts(vd, vq, th)):
            half = ARR - volts_to_duty(v) - MARGIN_COUNTS   # valid +/- this many counts
            d = abs(sample_at[i]) - half                    # >0 = outside window
            if d > 0:
                viol[i] += 1
            worst[i] = max(worst[i], d)
    print(f'{path}   ticks={n}   PWM_SAMPLE_OFFSET={offset}')
    print(f'  sample instants vs peak (counts): A {sample_at[0]:+d}  B {sample_at[1]:+d}  C {sample_at[2]:+d}')
    for i, name in enumerate('ABC'):
        pct = 100.0 * viol[i] / n if n else 0.0
        print(f'  {name}: {viol[i]:7d} outside ({pct:5.2f}%)   worst overshoot {worst[i]:+7.1f} counts'
              f' ({worst[i]/100.0:+.2f} us)')

if __name__ == '__main__':
    main(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 168)
