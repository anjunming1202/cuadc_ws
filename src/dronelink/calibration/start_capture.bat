@echo off
set OPENCV_FFMPEG_CAPTURE_OPTIONS=protocol_whitelist;file,udp,rtp
python "%USERPROFILE%\Desktop\capture_calibration.py"
