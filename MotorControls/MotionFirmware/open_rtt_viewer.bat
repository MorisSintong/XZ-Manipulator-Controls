@echo off
set RTT_EXE="C:\Program Files\SEGGER\JLink_V978\JLinkRTTViewer.exe"

if not exist %RTT_EXE% (
    echo [ERROR] JLinkRTTViewer.exe not found at %RTT_EXE%
    exit /b 1
)

echo [INFO] Launching SEGGER J-Link RTT Viewer for STM32F446RE...
start "" %RTT_EXE% --device STM32F446RE --interface SWD --speed 4000
