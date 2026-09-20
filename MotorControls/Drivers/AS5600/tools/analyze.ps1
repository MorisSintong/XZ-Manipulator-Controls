param(
    [string]$Cppcheck = 'cppcheck',
    [string]$Python = 'python',
    [switch]$RawFindings
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Push-Location $root
try {
    $output = Join-Path $root 'build-analysis'
    New-Item -ItemType Directory -Path $output -Force | Out-Null
    $arguments = @(
        '--std=c99',
        '--platform=arm32-wchar_t4',
        '--enable=warning,style,performance,portability',
        '--check-level=exhaustive',
        '--inconclusive',
        '--error-exitcode=1',
        '--template=gcc',
        '--addon=misra',
        "--addon-python=$Python",
        "--cppcheck-build-dir=$output",
        '--suppress=*:*fakes*',
        '-Iinclude',
        '-Iports\stm32_hal',
        '-Iports\cmsis_rtos2',
        '-Itests\fakes'
    )
    if (-not $RawFindings) {
        $arguments += '--suppressions-list=tools\cppcheck-suppressions.txt'
    }
    $arguments += @(
        'src\as5600.c',
        'ports\stm32_hal\as5600_stm32_hal.c',
        'ports\cmsis_rtos2\as5600_cmsis_rtos2.c'
    )
    $logName = if ($RawFindings) { 'raw-findings.txt' } else { 'analysis.txt' }
    & $Cppcheck @arguments 2>&1 | Tee-Object -FilePath (Join-Path $output $logName)
    if ($LASTEXITCODE -ne 0) {
        throw "Static analysis returned $LASTEXITCODE; inspect $output\$logName"
    }
}
finally {
    Pop-Location
}
