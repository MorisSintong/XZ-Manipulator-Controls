[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$Qemu,
    [string]$Image = (Join-Path $PSScriptRoot 'build\as5600_rtos.elf'),
    [string]$LogPath = (Join-Path $PSScriptRoot 'build\qemu.log'),
    [ValidateRange(1, 120)][int]$TimeoutSeconds = 30
)

$ErrorActionPreference = 'Stop'
$Qemu = (Get-Command $Qemu -ErrorAction Stop).Source
$Image = (Resolve-Path -LiteralPath $Image).Path
$start = [System.Diagnostics.ProcessStartInfo]::new()
$start.FileName = $Qemu
$start.UseShellExecute = $false
$start.CreateNoWindow = $true
$start.RedirectStandardOutput = $true
$start.RedirectStandardError = $true
foreach ($argument in @(
    '-M', 'mps2-an386', '-cpu', 'cortex-m4',
    '-display', 'none', '-serial', 'none', '-monitor', 'none',
    '-nic', 'none', '-no-reboot',
    '-icount', 'shift=3,align=off,sleep=off',
    '-semihosting-config', 'enable=on,target=native',
    '-kernel', $Image
)) {
    $start.ArgumentList.Add($argument)
}

$process = [System.Diagnostics.Process]::new()
$process.StartInfo = $start
$started = $false
try {
    if (-not $process.Start()) { throw 'Could not start QEMU.' }
    $started = $true
    $stdout = $process.StandardOutput.ReadToEndAsync()
    $stderr = $process.StandardError.ReadToEndAsync()
    $expired = -not $process.WaitForExit($TimeoutSeconds * 1000)
    if ($expired) {
        if (-not $process.HasExited) {
            Stop-Process -Id $process.Id -Force -ErrorAction Stop
        }
        if (-not $process.WaitForExit(5000)) {
            throw 'QEMU did not terminate after the external watchdog stopped its PID.'
        }
    }
    $log = $stdout.GetAwaiter().GetResult() + $stderr.GetAwaiter().GetResult()
    $log | Set-Content -LiteralPath $LogPath -Encoding utf8
    Write-Output $log
    if ($expired) { throw "QEMU exceeded the $TimeoutSeconds-second external watchdog." }
    if (($process.ExitCode -ne 0) -or ($log -notmatch '(?m)^RESULT: PASS\r?$') -or
        ($log -match '(?m)^FAIL:|^RESULT: FAIL')) {
        throw "RTOS integration test failed (QEMU exit $($process.ExitCode)). See $LogPath."
    }
} finally {
    if ($started -and -not $process.HasExited) {
        Stop-Process -Id $process.Id -Force -ErrorAction Stop
    }
    $process.Dispose()
}
