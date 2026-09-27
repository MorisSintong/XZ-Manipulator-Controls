@echo off
cd /d "%~dp0\.."
echo.
echo ================================================================
echo  TEST KOMUNIKASI SERIAL UART KE ESP32 (HUSEIN TA)
echo ================================================================
echo.
set PYTHONIOENCODING=utf-8

:: Auto-detect Python: prefer active venv/conda, fallback to system
where python >nul 2>&1
if %errorlevel% equ 0 (
    set PYTHON_CMD=python
) else (
    echo [ERROR] Python tidak ditemukan. Pastikan Python 3.10+ terinstall dan ada di PATH.
    pause
    exit /b 1
)

%PYTHON_CMD% 04_Source_Code\test_uart_esp32.py --port AUTO --angle 180.0
echo.
pause
