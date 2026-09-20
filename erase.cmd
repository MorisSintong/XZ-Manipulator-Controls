@echo off
C:\tools\jlink\JLink.exe -device STM32F446RE -if SWD -speed 4000 -autoconnect 1 -CommanderScript "%~dp0erase.jlink"
