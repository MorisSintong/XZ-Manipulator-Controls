<#
.SYNOPSIS
    Standalone PowerShell launcher for STM32H723 Mock UART Protocol Test Suite.

.DESCRIPTION
    Executes the Python UART protocol verification suite against the in-process
    MockFirmwareServer (default) or an existing TCP UART endpoint.

.PARAMETER Mode
    Execution mode: 'mock' (default, in-process MockFirmwareServer) or 'connect' (existing TCP endpoint).

.PARAMETER HostAddress
    Target TCP host address (default: 127.0.0.1).

.PARAMETER Port
    Target TCP port (default: 12345).

.EXAMPLE
    .\run_emulation_tests.ps1
    .\run_emulation_tests.ps1 -Mode connect -Port 12345
#>

param(
    [ValidateSet("mock", "connect")]
    [string]$Mode = "mock",

    [string]$HostAddress = "127.0.0.1",

    [int]$Port = 12345,

    [string]$JsonReport = "$PSScriptRoot\test_results.json"
)

$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $ScriptDir))

Write-Host "=================================================================" -ForegroundColor Cyan
Write-Host "   STM32H723 MOCK UART PROTOCOL TEST LAUNCHER                    " -ForegroundColor White
Write-Host "=================================================================" -ForegroundColor Cyan
Write-Host " Mode        : $Mode" -ForegroundColor Yellow
Write-Host " Target      : ${HostAddress}:${Port}" -ForegroundColor Yellow
Write-Host " Repo Root   : $RepoRoot" -ForegroundColor Yellow
Write-Host " Report Path : $JsonReport" -ForegroundColor Yellow
Write-Host "=================================================================" -ForegroundColor Cyan

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
