@echo off
setlocal
cd /d "%~dp0"

where ffmpeg >nul 2>&1
if errorlevel 1 (
  echo ffmpeg not found on PATH.
  echo Install with: winget install --id Gyan.FFmpeg -e
  exit /b 1
)

where python >nul 2>&1
if errorlevel 1 (
  echo python not found on PATH.
  exit /b 1
)

echo Starting DroneLink localhost viewer on http://127.0.0.1:8765/
echo Open that URL in Chrome/Edge. Ctrl+C stops the server.
python "%~dp0localhost_viewer\server.py" --sdp "%~dp0DroneLink.sdp"
endlocal
