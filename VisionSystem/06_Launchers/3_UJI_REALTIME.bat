@echo off
cd /d "%~dp0\.."
echo.
echo ================================================================
echo  DETEKSI REAL-TIME KAMERA + UART ESP32
echo  Tugas Akhir Machine Vision - Husein Alhamid (4212301035)
echo ================================================================
echo.
set PYTHONIOENCODING=utf-8
"C:\Users\HUSEN\.conda\envs\depth-obstacle-detector\python.exe" 04_Source_Code\4_detect_realtime.py --source 0 --uart-port AUTO
echo.
pause
