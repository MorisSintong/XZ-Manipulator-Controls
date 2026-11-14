@echo off
setlocal
set "SCRIPT=%~dp0flash.jlink"

if /i "%~1"=="diagnostics"    set "SCRIPT=%~dp0flash_diagnostics.jlink"
if /i "%~1"=="motion_diagnostics" set "SCRIPT=%~dp0flash_diagnostics.jlink"
if /i "%~1"=="stallguard"     set "SCRIPT=%~1"
if /i "%~1"=="stallguard"     set "SCRIPT=%~dp0flash_stallguard_cli.jlink"
if /i "%~1"=="stallguard_cli" set "SCRIPT=%~dp0flash_stallguard_cli.jlink"
if /i "%~1"=="cli"            set "SCRIPT=%~dp0flash_stallguard_cli.jlink"
if /i "%~1"=="zbounce"        set "SCRIPT=%~dp0flash_zbounce.jlink"
if /i "%~1"=="z_bounce_emulation" set "SCRIPT=%~dp0flash_zbounce.jlink"
if /i "%~1"=="usb_cdc"        set "SCRIPT=%~dp0flash_usb_cdc.jlink"
if /i "%~1"=="usbcdc"         set "SCRIPT=%~dp0flash_usb_cdc.jlink"
if /i "%~1"=="cdc"            set "SCRIPT=%~dp0flash_usb_cdc.jlink"

echo [FLASH] Using J-Link script: %SCRIPT%
C:\tools\jlink\JLink.exe -device STM32H723ZE -if SWD -speed 4000 -autoconnect 1 -CommanderScript "%SCRIPT%"
