@echo off
set RTT_EXE="C:\Program Files\SEGGER\JLink_V978\JLinkRTTViewer.exe"

if not exist %RTT_EXE% (
    echo [ERROR] JLinkRTTViewer.exe not found at %RTT_EXE%
    exit /b 1
)

echo [INFO] Launching SEGGER J-Link RTT Viewer for STM32H723ZET6...
start "" %RTT_EXE% --device STM32H723ZE --interface SWD --speed 4000 --search-ranges "0x24000000 0x10000"
