@echo off
setlocal
REM Ground station bring-up: Wi-Fi connect, then ONE localhost viewer.
REM Do not run start_localhost_viewer.bat in another window at the same time.
cd /d "%~dp0"

echo Stopping leftover FFmpeg so UDP ports are free...
taskkill /IM ffmpeg.exe /F >nul 2>&1

call "%~dp0connect_dronelink.bat"
if errorlevel 1 (
  echo Wi-Fi ping timed out — starting viewer anyway.
  echo When DroneLink associates, refresh http://127.0.0.1:8765/
  echo Do not launch another bat while this window is open.
)

echo.
call "%~dp0start_localhost_viewer.bat"
endlocal
