@echo off
:: ============================================================================
:: LAUNCHER PENGUJIAN DETEKSI & SUDUT OFFLINE (NON-REALTIME)
:: Tugas Akhir: Quality Control Kapasitor Berdasarkan Polaritas
:: Husein Alhamid (4212301035)
:: ============================================================================

cd /d "%~dp0"

echo.
echo ================================================================
echo  PENGUJIAN BATCH DATASET TEST OFFLINE (NON-REALTIME)
echo  Model   : 03_Models\best.pt
echo  Dataset : 02_Dataset\Kapasitor_Labeling_1500.v1i.yolo26\test\images
echo ================================================================
echo.

set PYTHONIOENCODING=utf-8
"C:\Users\HUSEN\.conda\envs\depth-obstacle-detector\python.exe" 04_Source_Code\3_test_offline.py

echo.
pause
