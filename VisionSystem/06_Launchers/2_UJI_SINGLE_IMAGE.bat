@echo off
cd /d "%~dp0\.."
echo.
echo ================================================================
echo  PENGUJIAN CEPAT 1 GAMBAR SAMPEL
echo  Tugas Akhir Machine Vision - Husein Alhamid (4212301035)
echo ================================================================
echo.
set PYTHONIOENCODING=utf-8
:: Auto-detect Python: local venv > conda env > system Python
set PYTHON_CMD=python
if exist "%~dp0..\.venv\Scripts\python.exe" set PYTHON_CMD=%~dp0..\.venv\Scripts\python.exe
if defined CONDA_PREFIX if exist "%CONDA_PREFIX%\python.exe" set PYTHON_CMD=%CONDA_PREFIX%\python.exe
"%PYTHON_CMD%" 04_Source_Code\5_test_single_image.py
echo.
pause
