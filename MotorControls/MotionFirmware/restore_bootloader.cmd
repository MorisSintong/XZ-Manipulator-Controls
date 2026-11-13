@echo off
setlocal
cd /d "%~dp0"

set "HEX_FILE=%~dp0tools\btt_bootloader\Octopus-H723-bootloader-128KB.hex"
if not exist "%HEX_FILE%" (
    echo [ERROR] Official BTT bootloader hex not found at %HEX_FILE%
    exit /b 1
)

echo [RESTORE] Restoring official BTT 128 KiB Bootloader (Octopus-H723-bootloader-128KB.hex) to 0x08000000 via J-Link SWD...
C:\tools\jlink\JLink.exe -device STM32H723ZE -if SWD -speed 4000 -autoconnect 1 -CommanderScript "%~dp0restore_bootloader.jlink"
