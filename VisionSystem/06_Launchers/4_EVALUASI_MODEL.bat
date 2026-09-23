@echo off
cd /d "%~dp0\.."
echo.
echo ================================================================
echo  EVALUASI KUANTITATIF MODEL (mAP, Precision, Recall, F1)
echo  Tugas Akhir Machine Vision - Husein Alhamid (4212301035)
echo ================================================================
echo.
set PYTHONIOENCODING=utf-8
:: Auto-detect Python: local venv > conda env > system Python
set PYTHON_CMD=python
if exist "%~dp0..\.venv\Scripts\python.exe" set PYTHON_CMD=%~dp0..\.venv\Scripts\python.exe
if defined CONDA_PREFIX if exist "%CONDA_PREFIX%\python.exe" set PYTHON_CMD=%CONDA_PREFIX%\python.exe
"%PYTHON_CMD%" 04_Source_Code\2_evaluate.py --split test
echo.
pause
