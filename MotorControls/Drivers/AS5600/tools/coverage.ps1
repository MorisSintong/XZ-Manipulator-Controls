param(
    [string]$BuildDirectory = 'build-coverage',
    [string]$LlvmProfdata = 'llvm-profdata',
    [string]$LlvmCov = 'llvm-cov',
    [double]$MinimumBranchCoverage = 99.0
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Push-Location $root
try {
    $build = (Resolve-Path -LiteralPath $BuildDirectory).Path
    $profiles = @()
    foreach ($name in @('core', 'ports', 'concurrency')) {
        $path = Join-Path (Join-Path $build 'profiles') ($name + '.profraw')
        if (-not (Test-Path -LiteralPath $path)) {
            throw "Missing $path; build with AS5600_ENABLE_COVERAGE and run all three tests first."
        }
        $profiles += (Resolve-Path -LiteralPath $path).Path
    }
    $profile = Join-Path $build 'as5600.profdata'
    & $LlvmProfdata merge -sparse @profiles -o $profile
    if ($LASTEXITCODE -ne 0) {
        throw 'llvm-profdata merge failed.'
    }
    $executables = @()
    foreach ($name in @('as5600_test_core', 'as5600_test_ports', 'as5600_test_concurrency')) {
        $path = Join-Path $build ($name + '.exe')
        if (-not (Test-Path -LiteralPath $path)) {
            $path = Join-Path $build $name
        }
        $executables += (Resolve-Path -LiteralPath $path).Path
    }
    for ($index = 0; $index -lt $profiles.Count; ++$index) {
        if ((Get-Item -LiteralPath $profiles[$index]).LastWriteTimeUtc -lt
            (Get-Item -LiteralPath $executables[$index]).LastWriteTimeUtc) {
            throw 'A coverage profile predates its executable; rerun all three tests.'
        }
    }
    $arguments = @(
        $executables[0],
        '-object', $executables[1],
        '-object', $executables[2],
        '-instr-profile', $profile,
        'src\as5600.c',
        'ports\stm32_hal\as5600_stm32_hal.c',
        'ports\cmsis_rtos2\as5600_cmsis_rtos2.c'
    )
    & $LlvmCov report @arguments |
        Tee-Object -FilePath (Join-Path $build 'coverage-summary.txt')
    if ($LASTEXITCODE -ne 0) {
        throw 'llvm-cov report failed.'
    }
    $json = & $LlvmCov export -summary-only @arguments
    if ($LASTEXITCODE -ne 0) {
        throw 'llvm-cov export failed.'
    }
    $report = $json | ConvertFrom-Json
    if ($report.data[0].files.Count -ne 3) {
        throw 'Coverage must describe exactly the three production translation units.'
    }
    $totals = $report.data[0].totals
    if (($totals.lines.covered -ne $totals.lines.count) -or
        ($totals.functions.covered -ne $totals.functions.count) -or
        ($totals.branches.percent -lt $MinimumBranchCoverage)) {
        throw 'Production coverage is below the required line/function/branch thresholds.'
    }
}
finally {
    Pop-Location
}
