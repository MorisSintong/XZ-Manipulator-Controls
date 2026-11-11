param(
    [string]$HostCompiler = "gcc",
    [string]$ArmCompiler = "arm-none-eabi-gcc"
)

$ErrorActionPreference = "Stop"
$tests = $PSScriptRoot
$hostBuild = Join-Path $tests "build-host"
$armBuild = Join-Path $tests "build-arm"
$app = Split-Path $tests -Parent
$coreInc = Join-Path (Split-Path (Split-Path $app -Parent) -Parent) "Core\Inc"
$hostExe = (Get-Command $HostCompiler -ErrorAction Stop).Source
$armExe = (Get-Command $ArmCompiler -ErrorAction Stop).Source

& cmake -S $tests -B $hostBuild -G Ninja "-DCMAKE_C_COMPILER=$hostExe" "-DCMAKE_BUILD_TYPE=Debug"
if ($LASTEXITCODE -ne 0) { throw "Host configuration failed" }
& cmake --build $hostBuild
if ($LASTEXITCODE -ne 0) { throw "Host build failed" }
& ctest --test-dir $hostBuild --output-on-failure
if ($LASTEXITCODE -ne 0) { throw "Host tests failed" }

$halFreeSources = @(
    "crc16_ccitt.c", "vision_frame.c", "encoder_unwrap.c", "motion_units.c",
    "motion_config.c", "diag_record.c", "motion_profile.c", "step_engine.c",
    "encoder_sampler.c", "driver_evidence.c", "uart_transport.c", "homing.c",
    "motion_executor.c", "command_dispatch.c"
)

New-Item -ItemType Directory -Force $armBuild | Out-Null
foreach ($name in $halFreeSources) {
    $source = Get-Item (Join-Path (Join-Path $app "Src") $name)
    $object = Join-Path $armBuild ($source.BaseName + ".o")
    & $armExe -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard `
        -std=c11 -Wall -Wextra -Werror -Wconversion -Wsign-conversion `
        -I (Join-Path $app "Inc") -I $coreInc -c $source.FullName -o $object
    if ($LASTEXITCODE -ne 0) { throw "ARM compilation failed: $($source.Name)" }
    Write-Output "ARM compile PASS: $($source.Name)"
}
