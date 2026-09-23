@echo off
cd /d "%~dp0\.."
echo.
echo ================================================================
echo  TEST KOMUNIKASI SERIAL UART KE ESP32 (HUSEIN TA)
echo ================================================================
echo.
set PYTHONIOENCODING=utf-8
:: Auto-detect Python: local venv > conda env > system Python
set PYTHON_CMD=python
if exist "%~dp0..\.venv\Scripts\python.exe" set PYTHON_CMD=%~dp0..\.venv\Scripts\python.exe
if defined CONDA_PREFIX if exist "%CONDA_PREFIX%\python.exe" set PYTHON_CMD=%CONDA_PREFIX%\python.exe
"%PYTHON_CMD%" 04_Source_Code\test_uart_esp32.py --port AUTO --angle 180.0
echo.
pause
