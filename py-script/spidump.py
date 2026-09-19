#!/usr/bin/env python3
"""Clock N 32-byte frames off the STM32 and show what actually arrives.

Matches the capture tool's link settings: SPI mode 1 (CPOL=0, CPHA=1), no
hardware CS, manual CS on GPIO25 (see foc-sysid/src/spi.cpp).

For each frame: raw hex, the CRC it carries, the CRC computed over its first 28
bytes, and for a mismatch, a search over all byte rotations -- which separates
"corrupt content" from "frame offset by N bytes".
"""
import spidev, lgpio, sys, time, collections

CS_PIN = 25
TAB = []
for i in range(256):
    c = i << 8
    for _ in range(8):
        c = ((c << 1) ^ 0x1021) & 0xFFFF if (c & 0x8000) else (c << 1) & 0xFFFF
    TAB.append(c)

def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc = ((crc << 8) ^ TAB[(crc >> 8) ^ b]) & 0xFFFF
    return crc

n = int(sys.argv[1]) if len(sys.argv) > 1 else 12
h = lgpio.gpiochip_open(0)
try:
    lgpio.gpio_free(h, CS_PIN)
except Exception:
    pass
lgpio.gpio_claim_output(h, 0, CS_PIN, 1)

spi = spidev.SpiDev(); spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.mode = 1
spi.no_cs = True

good = 0
rot_hits = collections.Counter()
for k in range(n):
    lgpio.gpio_write(h, CS_PIN, 0)
    rx = bytes(spi.xfer2([0x00] * 32))
    lgpio.gpio_write(h, CS_PIN, 1)
    time.sleep(0.001)
    carried = int.from_bytes(rx[28:30], 'little')
    calc = crc16(rx[:28])
    if carried == calc:
        good += 1
        print(f'{k:3d} OK  t={int.from_bytes(rx[0:4],"little"):10d} flags=0x{int.from_bytes(rx[26:28],"little"):04X} '
              f'test_id={int.from_bytes(rx[30:32],"little")}  {rx.hex()}')
    else:
        print(f'{k:3d} BAD carried=0x{carried:04X} calc=0x{calc:04X}  {rx.hex()}')
        for r in range(1, 32):
            rot = rx[r:] + rx[:r]
            if int.from_bytes(rot[28:30], 'little') == crc16(rot[:28]):
                rot_hits[r] += 1
                print(f'      -> checks out rotated by {r} bytes')
                break
spi.close(); lgpio.gpiochip_close(h)
print(f'\n{good}/{n} frames pass CRC')
if rot_hits:
    print('rotations that fixed a frame:', rot_hits.most_common())
