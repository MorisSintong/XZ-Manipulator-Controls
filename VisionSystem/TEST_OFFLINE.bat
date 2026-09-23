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
:: Auto-detect Python: local venv > conda env > system Python
set PYTHON_CMD=python
if exist "%~dp0.venv\Scripts\python.exe" set PYTHON_CMD=%~dp0.venv\Scripts\python.exe
if defined CONDA_PREFIX if exist "%CONDA_PREFIX%\python.exe" set PYTHON_CMD=%CONDA_PREFIX%\python.exe
"%PYTHON_CMD%" 04_Source_Code\3_test_offline.py

echo.
pause
