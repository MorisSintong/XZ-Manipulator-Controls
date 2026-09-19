@echo off
cd /d "%~dp0\.."
echo.
echo ================================================================
echo  EVALUASI KUANTITATIF MODEL (mAP, Precision, Recall, F1)
echo  Tugas Akhir Machine Vision - Husein Alhamid (4212301035)
echo ================================================================
echo.
set PYTHONIOENCODING=utf-8
"C:\Users\HUSEN\.conda\envs\depth-obstacle-detector\python.exe" 04_Source_Code\2_evaluate.py --split test
echo.
pause
