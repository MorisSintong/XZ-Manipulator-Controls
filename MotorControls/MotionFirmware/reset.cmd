@echo off
C:\tools\jlink\JLink.exe -device STM32H723ZE -if SWD -speed 4000 -autoconnect 1 -CommanderScript "%~dp0reset.jlink"
