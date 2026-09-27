# Simple Serial Monitor for STM32 TMC2240 diagnostics on COM9
param(
    [string]$PortName = "COM9",
    [int]$BaudRate = 115200
)

try {
    $port = New-Object System.IO.Ports.SerialPort $PortName, $BaudRate, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One
    $port.ReadTimeout = 1000
    $port.Open()
    Write-Host "[INFO] Connected to $PortName at $BaudRate baud." -ForegroundColor Cyan
    Write-Host "[INFO] Press Ctrl+C to stop listening.`r`n" -ForegroundColor DarkGray

    while ($true) {
        try {
            $line = $port.ReadLine()
            Write-Host $line
        } catch [System.TimeoutException] {
            # normal read timeout, continue loop
        }
    }
} catch {
    Write-Host "[ERROR] Could not open $PortName: $_" -ForegroundColor Red
} finally {
    if ($null -ne $port -and $port.IsOpen) {
        $port.Close()
    }
}
