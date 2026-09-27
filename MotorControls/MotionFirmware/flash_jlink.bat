@echo off
set JLINK_EXE="C:\Program Files\SEGGER\JLink_V978\JLink.exe"

if not exist %JLINK_EXE% (
    echo [ERROR] JLink.exe not found at %JLINK_EXE%
    exit /b 1
)

echo [INFO] Flashing MotionFirmware.hex to STM32F446RE via J-Link SWD...
%JLINK_EXE% -device STM32F446RE -if SWD -speed 4000 -autoconnect 1 -CommanderScript flash.jlink

if %ERRORLEVEL% equ 0 (
    echo [SUCCESS] Flashing complete! Target reset and running.
) else (
    echo [ERROR] Flashing failed with exit code %ERRORLEVEL%
)
