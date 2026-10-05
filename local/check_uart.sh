#!/bin/bash
echo '===ALIASES==='
for a in serial0 serial1 uart0 uart1 uart10; do
  printf '%s: ' "$a"
  cat "/proc/device-tree/aliases/$a" 2>/dev/null
  echo
done
echo '===AMA0==='
readlink -f /sys/class/tty/ttyAMA0/device
echo '===AMA10==='
readlink -f /sys/class/tty/ttyAMA10/device
echo '===CONFIG==='
grep -nE 'uart|serial|enable_uart|dtoverlay' /boot/firmware/config.txt
echo '===OVERLAY uart0==='
ls /boot/firmware/overlays/uart0* /boot/firmware/overlays/disable-bt* 2>/dev/null
echo '===PINCTRL==='
command -v pinctrl
pinctrl get 14 15 2>/dev/null || true
