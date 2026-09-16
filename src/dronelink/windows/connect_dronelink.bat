@echo off
setlocal
REM Fast reconnect to DroneLink AP on the USB Wi-Fi adapter.
REM Edit IFACE if your adapter name is not "WLAN 2" (check: netsh wlan show interfaces).

set "IFACE=WLAN 2"
set "SSID=DroneLink"
set /a MAX_TRIES=30

echo Connecting interface "%IFACE%" to SSID "%SSID%" ...
for /l %%i in (1,1,%MAX_TRIES%) do (
  netsh wlan connect name=%SSID% interface="%IFACE%" >nul 2>&1
  ping -n 1 -w 500 192.168.50.1 >nul 2>&1
  if not errorlevel 1 (
    echo Connected: ping 192.168.50.1 OK
    exit /b 0
  )
  echo attempt %%i/%MAX_TRIES% ...
  ping -n 2 127.0.0.1 >nul
)

echo Failed to reach 192.168.50.1 within timeout.
echo Check: USB-AC56 plugged, static IP 192.168.50.2, SSID=%SSID%, interface name.
exit /b 1
