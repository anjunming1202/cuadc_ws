#!/usr/bin/env python3
"""Dump raw UART bytes for a few seconds. Does not arm."""
import sys
import time
import serial

dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyAMA0"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 57600
sec = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0
print(f"dump {dev} @{baud} {sec}s", flush=True)
s = serial.Serial(dev, baud, timeout=0.2)
t0 = time.time()
n = 0
while time.time() - t0 < sec:
    b = s.read(256)
    if b:
        n += len(b)
        print(b[:40].hex(), flush=True)
print(f"bytes {n}", flush=True)
s.close()
