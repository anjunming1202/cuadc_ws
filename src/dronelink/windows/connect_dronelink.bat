@echo off
setlocal EnableDelayedExpansion
REM Connect WLAN 2 to DroneLink. Do not disconnect first — that delays Windows
REM association while the Pi AP is still coming up.
REM Edit IFACE if needed:  netsh wlan show interfaces

set "IFACE=WLAN 2"
set "SSID=DroneLink"
set /a MAX_TRIES=90

echo [%TIME%] Connecting "%IFACE%" -> "%SSID%" ...

ping -n 1 -w 800 192.168.50.1 >nul 2>&1
if not errorlevel 1 (
  echo [%TIME%] Already up: ping 192.168.50.1 OK
  exit /b 0
)

netsh wlan connect name=%SSID% ssid=%SSID% interface="%IFACE%" >nul 2>&1

for /l %%i in (1,1,%MAX_TRIES%) do (
  ping -n 1 -w 800 192.168.50.1 >nul 2>&1
  if not errorlevel 1 (
    echo [%TIME%] Connected: ping 192.168.50.1 OK ^(attempt %%i^)
    exit /b 0
  )
  set /a MOD=%%i %% 8
  if !MOD! equ 0 (
    echo [%TIME%] attempt %%i/%MAX_TRIES% — asking Windows to connect...
    netsh wlan connect name=%SSID% ssid=%SSID% interface="%IFACE%" >nul 2>&1
  ) else (
    echo [%TIME%] attempt %%i/%MAX_TRIES% waiting for 192.168.50.1 ...
  )
  ping -n 2 127.0.0.1 >nul
)

echo [%TIME%] Timed out pinging 192.168.50.1
echo If WLAN 2 later shows DroneLink, just wait — do NOT start a second viewer.
echo Check: USB-AC56 plugged, static IP 192.168.50.2, Pi powered.
exit /b 1
