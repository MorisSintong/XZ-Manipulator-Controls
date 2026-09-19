@echo off
cd /d "%~dp0\.."
echo.
echo ================================================================
echo  PENGUJIAN BATCH DATASET TEST OFFLINE (NON-REALTIME)
echo  Tugas Akhir Machine Vision - Husein Alhamid (4212301035)
echo ================================================================
echo.
set PYTHONIOENCODING=utf-8
"C:\Users\HUSEN\.conda\envs\depth-obstacle-detector\python.exe" 04_Source_Code\3_test_offline.py
echo.
pause
