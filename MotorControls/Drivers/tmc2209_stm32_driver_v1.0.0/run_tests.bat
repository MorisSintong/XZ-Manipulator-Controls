@echo off
setlocal
set "TMC_BUILD_DIR=%~dp0..\.build\tmc2209-host"
if not "%~1"=="" set "TMC_BUILD_DIR=%~f1"

cmake -S "%~dp0Tests" -B "%TMC_BUILD_DIR%" -G Ninja -DCMAKE_C_COMPILER=clang -DCMAKE_BUILD_TYPE=Debug
if errorlevel 1 exit /b 1
cmake --build "%TMC_BUILD_DIR%" --parallel 2
if errorlevel 1 exit /b 1
ctest --test-dir "%TMC_BUILD_DIR%" --output-on-failure --no-tests=error --timeout 60
if errorlevel 1 exit /b 1

echo Host tests passed. This does not qualify STM32 hardware or electrical behavior.
exit /b 0
