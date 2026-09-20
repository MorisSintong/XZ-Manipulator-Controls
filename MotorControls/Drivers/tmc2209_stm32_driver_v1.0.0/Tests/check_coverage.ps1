param(
    [Parameter(Mandatory = $true)][string]$CoverageJson,
    [Parameter(Mandatory = $true)][string]$OutputCsv
)
$ErrorActionPreference = 'Stop'
$report = Get-Content -LiteralPath $CoverageJson -Raw | ConvertFrom-Json
$groups = [ordered]@{
    core = @('tmc2209.c', 'tmc2209_ll.c')
    hal = @('tmc2209_port_hal.c')
    os_none = @('tmc2209_os_none.c')
    os_freertos = @('tmc2209_os_freertos.c')
    os_combined = @('tmc2209_os_none.c', 'tmc2209_os_freertos.c')
}
$rows = foreach ($group in $groups.GetEnumerator()) {
    $members = @($report.data.files | Where-Object {
        [IO.Path]::GetFileName($_.filename) -in $group.Value
    })
    if ($members.Count -ne $group.Value.Count) {
        throw "Missing or duplicate production source coverage in $($group.Key)"
    }
    $lines = ($members | ForEach-Object { $_.summary.lines.count } | Measure-Object -Sum).Sum
    $lineHits = ($members | ForEach-Object { $_.summary.lines.covered } | Measure-Object -Sum).Sum
    $branches = ($members | ForEach-Object { $_.summary.branches.count } | Measure-Object -Sum).Sum
    $branchHits = ($members | ForEach-Object { $_.summary.branches.covered } | Measure-Object -Sum).Sum
    if ($lines -le 0 -or $branches -le 0) { throw "Empty production coverage group $($group.Key)" }
    $linePercent = 100.0 * $lineHits / $lines
    $branchPercent = 100.0 * $branchHits / $branches
    if ($linePercent -lt 95.0 -or $branchPercent -lt 90.0) {
        throw "Coverage gate failed: $($group.Key): lines=$linePercent, branches=$branchPercent"
    }
    [PSCustomObject]@{
        Group = $group.Key
        LinesCovered = $lineHits
        LinesTotal = $lines
        LinePercent = $linePercent
        BranchesCovered = $branchHits
        BranchesTotal = $branches
        BranchPercent = $branchPercent
    }
}
$rows | Export-Csv -NoTypeInformation -LiteralPath $OutputCsv
$rows | Format-Table -AutoSize
