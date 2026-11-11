@echo off
setlocal
set "SCRIPT=%~dp0flash.jlink"

if /i "%~1"=="diagnostics"    set "SCRIPT=%~dp0flash_diagnostics.jlink"
if /i "%~1"=="motion_diagnostics" set "SCRIPT=%~dp0flash_diagnostics.jlink"
if /i "%~1"=="stallguard"     set "SCRIPT=%~dp0flash_stallguard_cli.jlink"
if /i "%~1"=="stallguard_cli" set "SCRIPT=%~dp0flash_stallguard_cli.jlink"
if /i "%~1"=="cli"            set "SCRIPT=%~dp0flash_stallguard_cli.jlink"
if /i "%~1"=="zbounce"        set "SCRIPT=%~dp0flash_zbounce.jlink"
if /i "%~1"=="z_bounce_emulation" set "SCRIPT=%~dp0flash_zbounce.jlink"

echo [FLASH] Using J-Link script: %SCRIPT%
C:\tools\jlink\JLink.exe -device STM32F446RE -if SWD -speed 4000 -autoconnect 1 -CommanderScript "%SCRIPT%"
