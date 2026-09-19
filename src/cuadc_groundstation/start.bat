@echo off
REM cuadc_groundstation localhost viewer —— 在地面笔记本(Windows)上跑。
REM 前提:笔记本已连 DroneLink、静态 IP 192.168.50.2;PATH 上有 Python 与 ffmpeg。
REM 从本 bat 所在目录起 server.py,浏览器打开 http://127.0.0.1:8765/。
cd /d "%~dp0"
python server.py %*
