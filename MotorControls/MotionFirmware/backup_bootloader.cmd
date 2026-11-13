@echo off
setlocal
cd /d "%~dp0"
if exist "bootloader_btt_octopus_pro_128k.bin" (
    echo [WARN] bootloader_btt_octopus_pro_128k.bin already exists!
    echo        Delete or rename it first to avoid overwriting your original BTT bootloader backup.
    exit /b 1
)
echo [BACKUP] Dumping 128 KiB BTT Bootloader (0x08000000..0x0801FFFF) and 512 KiB Flash via J-Link SWD...
C:\tools\jlink\JLink.exe -device STM32H723ZE -if SWD -speed 4000 -autoconnect 1 -CommanderScript "%~dp0backup_bootloader.jlink"
if %ERRORLEVEL% equ 0 (
    echo [SUCCESS] Saved bootloader_btt_octopus_pro_128k.bin and flash_full_backup_512k.bin
) else (
    echo [ERROR] J-Link backup failed with exit code %ERRORLEVEL%
)
