@echo off
start "" "C:\tools\jlink\JLinkRTTViewer.exe" --device STM32H723ZE --interface SWD --speed 4000 --search-ranges "0x24000000 0x10000"
