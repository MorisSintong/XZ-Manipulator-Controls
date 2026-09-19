@echo off
cd /d "%~dp0\.."
echo.
echo ================================================================
echo  TEST KOMUNIKASI SERIAL UART KE ESP32 (HUSEIN TA)
echo ================================================================
echo.
set PYTHONIOENCODING=utf-8
"C:\Users\HUSEN\.conda\envs\depth-obstacle-detector\python.exe" 04_Source_Code\test_uart_esp32.py --port AUTO --angle 180.0
echo.
pause
