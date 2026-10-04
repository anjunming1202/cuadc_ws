#!/usr/bin/env python3
from pathlib import Path
p = Path("/proc/device-tree/aliases")
for n in (
    "uart0", "uart1", "uart2", "uart3", "uart4", "uart10",
    "serial0", "serial1", "serial10",
):
    f = p / n
    if f.exists():
        print(n, f.read_bytes().split(b"\x00")[0].decode())
    else:
        print(n, "MISSING")
