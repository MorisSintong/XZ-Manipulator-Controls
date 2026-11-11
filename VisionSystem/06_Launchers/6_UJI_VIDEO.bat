@echo off
chcp 65001 >nul 2>&1
title Pengujian Video QC Kapasitor + STM32 UART - Husein Alhamid TA

echo ============================================================
echo   PENGUJIAN VIDEO QC KAPASITOR + STM32 UART
echo   Tugas Akhir: Husein Alhamid (4212301035)
echo ============================================================
echo.

:: Minta input path video dari pengguna
set /p VIDEO_PATH="  Masukkan path file video (contoh: D:\video\test.mp4): "

if "%VIDEO_PATH%"=="" (
    echo [ERROR] Path video tidak boleh kosong!
    pause
    exit /b 1
)

echo.
echo   Pilih mode pengujian:
echo   [1] Dengan STM32 UART (AUTO detect port)
echo   [2] Dengan STM32 UART (Pilih port manual)
echo   [3] Tanpa UART / Offline (--no-uart)
echo.
set /p MODE="  Pilihan (1/2/3): "

cd /d "%~dp0\.."

:: Auto-detect Python: prefer project .venv, then active venv/conda/system
if exist ".venv\Scripts\python.exe" (
    set "PYTHON_CMD=.venv\Scripts\python.exe"
) else (
    where python >nul 2>&1
    if %errorlevel% equ 0 (
        set PYTHON_CMD=python
    ) else (
        echo [ERROR] Python tidak ditemukan. Pastikan Python 3.10+ terinstall dan ada di PATH.
        pause
        exit /b 1
    )
)

if "%MODE%"=="1" (
    echo.
    echo   [INFO] Menjalankan dengan UART AUTO...
    %PYTHON_CMD% "04_Source_Code\6_test_video.py" --video "%VIDEO_PATH%" --uart-port AUTO --save-video
) else if "%MODE%"=="2" (
    set /p COM_PORT="  Masukkan port COM STM32 (contoh: COM9): "
    echo.
    echo   [INFO] Menjalankan dengan UART port %COM_PORT%...
    %PYTHON_CMD% "04_Source_Code\6_test_video.py" --video "%VIDEO_PATH%" --uart-port %COM_PORT% --save-video
) else (
    echo.
    echo   [INFO] Menjalankan mode OFFLINE (tanpa UART)...
    %PYTHON_CMD% "04_Source_Code\6_test_video.py" --video "%VIDEO_PATH%" --no-uart --save-video
)

echo.
echo ============================================================
echo   SELESAI. Hasil tersimpan di 05_Hasil_Pengujian\video_test_results\
echo ============================================================
pause
