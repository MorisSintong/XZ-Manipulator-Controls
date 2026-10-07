<#
.SYNOPSIS
    Standalone PowerShell launcher for STM32F446RE UART Emulation Automated Test Suite.

.DESCRIPTION
    Builds the STM32 firmware (if requested or missing) and executes the comprehensive
    Python test suite against Renode emulation or mock server.

.PARAMETER Mode
    Emulation mode: 'auto' (default), 'renode', or 'mock'.

.PARAMETER HostAddress
    Target TCP host address (default: 127.0.0.1).

.PARAMETER Port
    Target TCP port (default: 12345).

.PARAMETER BuildFirmware
    Switch to force re-compilation of MotionFirmware prior to running tests.

.EXAMPLE
    .\run_emulation_tests.ps1 -Mode auto
    .\run_emulation_tests.ps1 -Mode mock
    .\run_emulation_tests.ps1 -Mode renode -BuildFirmware
#>

param(
    [ValidateSet("auto", "renode", "mock")]
    [string]$Mode = "auto",

    [string]$HostAddress = "127.0.0.1",

    [int]$Port = 12345,

    [string]$JsonReport = "$PSScriptRoot\test_results.json",

    [switch]$BuildFirmware
)

$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Split-Path -Parent (Split-Path -Parent $ScriptDir)

Write-Host "=================================================================" -ForegroundColor Cyan
Write-Host "   STM32F446RE UART EMULATION TEST LAUNCHER                     " -ForegroundColor White
Write-Host "=================================================================" -ForegroundColor Cyan
Write-Host " Mode        : $Mode" -ForegroundColor Yellow
Write-Host " Target      : ${HostAddress}:${Port}" -ForegroundColor Yellow
Write-Host " Repo Root   : $RepoRoot" -ForegroundColor Yellow
Write-Host " Report Path : $JsonReport" -ForegroundColor Yellow
Write-Host "=================================================================" -ForegroundColor Cyan

# 1. Firmware ELF Verification / Build
$ElfPath = Join-Path $RepoRoot "MotorControls\MotionFirmware\build\Debug\MotionFirmware.elf"
if ($BuildFirmware -or (-not (Test-Path $ElfPath))) {
    Write-Host "[BUILD] Compiling STM32 MotionFirmware..." -ForegroundColor Green
    $BuildDir = Join-Path $RepoRoot "MotorControls\MotionFirmware\build\Debug"
    $SrcDir   = Join-Path $RepoRoot "MotorControls\MotionFirmware"

    if (Get-Command cmake -ErrorAction SilentlyContinue) {
        cmake -B $BuildDir -S $SrcDir
        if ($LASTEXITCODE -ne 0) {
            Write-Error "CMake configuration failed!"
            exit 1
        }
        ninja -C $BuildDir
        if ($LASTEXITCODE -ne 0) {
            Write-Error "Ninja compilation failed!"
            exit 1
        }
        Write-Host "[BUILD] Firmware compiled successfully -> $ElfPath" -ForegroundColor Green
    } else {
        Write-Warning "CMake not found in PATH; skipping firmware build."
    }
}

# 2. Run Python Automated Test Suite
$TestScript = Join-Path $ScriptDir "test_stm32_uart_emulation.py"
Write-Host "[EXEC] Running Python test suite..." -ForegroundColor Cyan

$PythonExe = "python"
if (-not (Get-Command $PythonExe -ErrorAction SilentlyContinue)) {
    Write-Error "Python executable not found in PATH!"
    exit 1
}

& $PythonExe $TestScript --host $HostAddress --port $Port --mode $Mode --json-report $JsonReport
$ExitCode = $LASTEXITCODE

if ($ExitCode -eq 0) {
    Write-Host "[RESULT] ALL EMULATION TESTS PASSED SUCCESSFULLY!" -ForegroundColor Green
} else {
    Write-Host "[RESULT] TEST SUITE FAILED WITH EXIT CODE $ExitCode" -ForegroundColor Red
}

exit $ExitCode
