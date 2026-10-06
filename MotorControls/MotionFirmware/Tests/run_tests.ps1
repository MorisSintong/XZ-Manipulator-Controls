param(
    [string]$HostCompiler = "gcc",
    [string]$ArmCompiler = "arm-none-eabi-gcc"
)

$ErrorActionPreference = "Stop"
$tests = $PSScriptRoot
$hostBuild = Join-Path $tests "build-host"
$armBuild = Join-Path $tests "build-arm"
$app = Join-Path (Split-Path $tests -Parent) "App"
$hostExe = (Get-Command $HostCompiler -ErrorAction Stop).Source
$armExe = (Get-Command $ArmCompiler -ErrorAction Stop).Source

& cmake -S $tests -B $hostBuild -G Ninja "-DCMAKE_C_COMPILER=$hostExe" "-DCMAKE_BUILD_TYPE=Debug"
if ($LASTEXITCODE -ne 0) { throw "Host configuration failed" }
& cmake --build $hostBuild
if ($LASTEXITCODE -ne 0) { throw "Host build failed" }
& ctest --test-dir $hostBuild --output-on-failure
if ($LASTEXITCODE -ne 0) { throw "Host tests failed" }

New-Item -ItemType Directory -Force $armBuild | Out-Null
foreach ($source in Get-ChildItem (Join-Path $app "Src") -Filter "*.c") {
    $object = Join-Path $armBuild ($source.BaseName + ".o")
    & $armExe -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard `
        -std=c11 -Wall -Wextra -Werror -Wconversion -Wsign-conversion `
        -I (Join-Path $app "Inc") -c $source.FullName -o $object
    if ($LASTEXITCODE -ne 0) { throw "ARM compilation failed: $($source.Name)" }
    Write-Output "ARM compile PASS: $($source.Name)"
}
