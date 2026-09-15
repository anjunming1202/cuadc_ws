@echo off
"C:\Program Files\VideoLAN\VLC\vlc.exe" --network-caching=20 --clock-jitter=0 --clock-synchro=0 --drop-late-frames --skip-frames "%USERPROFILE%\Desktop\DroneLink.sdp"
