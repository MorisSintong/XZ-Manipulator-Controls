@echo off
cd /d "%~dp0\.."
echo.
echo ================================================================
echo  PENGUJIAN BATCH DATASET TEST OFFLINE (NON-REALTIME)
echo  Tugas Akhir Machine Vision - Husein Alhamid (4212301035)
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

%PYTHON_CMD% 04_Source_Code\3_test_offline.py
echo.
pause
