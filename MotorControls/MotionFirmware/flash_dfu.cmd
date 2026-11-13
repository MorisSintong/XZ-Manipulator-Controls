@echo off
setlocal
cd /d "%~dp0"

set "PROG_CLI=C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
if not exist "%PROG_CLI%" (
    echo [ERROR] STM32_Programmer_CLI.exe not found at "%PROG_CLI%"
    exit /b 1
)

set "TARGET_BIN=%~dp0build\Release\MotionFirmware.bin"
if not exist "%TARGET_BIN%" (
    set "TARGET_BIN=%~dp0build\Debug\MotionFirmware.bin"
)
if not exist "%TARGET_BIN%" (
    echo [ERROR] No MotionFirmware.bin found in build\Release or build\Debug.
    echo Please build first: cmake --build --preset Release
    exit /b 1
)

echo =========================================================================
echo  Flashing STM32H723 via USB-C DFU (STM32CubeProgrammer CLI)
echo =========================================================================
echo Target Binary: %TARGET_BIN%
echo Target Flash Address: 0x08000000
echo.
echo Make sure the BOOT0 jumper is installed and the board is plugged via USB-C!
echo.

"%PROG_CLI%" -c port=usb1 -w "%TARGET_BIN%" 0x08000000 -v -s

if %ERRORLEVEL% equ 0 (
    echo.
    echo [PASS] Flash complete! 
    echo Please remove the BOOT0 jumper and press the RESET button.
) else (
    echo.
    echo [FAIL] Could not connect to DFU USB device.
    echo Checklist:
    echo  1. Is the BOOT0 jumper bridging BOOT0 to 3.3V?
    echo  2. Did you press the physical RESET button after plugging in USB-C?
    echo  3. Is the USB-C cable a data cable (not charge-only)?
)
