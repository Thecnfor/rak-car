$paths = @(
    @{ N = "pipCache";   P = "C:\Users\$env:USERNAME\AppData\Local\pip\cache";       R = "very-low"; MinAgeDays = 0 },
    @{ N = "npmCache";   P = "C:\Users\$env:USERNAME\AppData\Local\npm-cache";        R = "very-low"; MinAgeDays = 0 },
    @{ N = "UserTemp";   P = "$env:TEMP";                                              R = "low";      MinAgeDays = 1 }
)

function Get-DirSize($p) {
    if (-not (Test-Path $p)) { return 0 }
    $s = (Get-ChildItem $p -Recurse -Force -ErrorAction SilentlyContinue |
          Where-Object { -not $_.PSIsContainer } |
          Measure-Object -Property Length -Sum).Sum
    if ($null -eq $s) { $s = 0 }
    return $s
}

$totalBefore = 0
$totalAfter = 0

foreach ($c in $paths) {
    if (-not (Test-Path $c.P)) {
        Write-Host "[skip] $($c.N) - not found" -ForegroundColor DarkGray
        continue
    }
    $before = Get-DirSize $c.P
    $totalBefore += $before
    Write-Host "[cleaning] $($c.N) - $([math]::Round($before/1MB,1)) MB ..." -ForegroundColor Yellow

    try {
        if ($c.MinAgeDays -gt 0) {
            Get-ChildItem $c.P -Recurse -Force -ErrorAction SilentlyContinue |
                Where-Object { -not $_.PSIsContainer -and $_.LastWriteTime -lt (Get-Date).AddDays(-$c.MinAgeDays) } |
                Remove-Item -Force -ErrorAction SilentlyContinue
        } else {
            Get-ChildItem $c.P -Recurse -Force -ErrorAction SilentlyContinue |
                Remove-Item -Recurse -Force -ErrorAction SilentlyContinue
        }
    } catch {
        Write-Host "  [warn] $($c.N): $($_.Exception.Message)" -ForegroundColor DarkYellow
    }

    $after = Get-DirSize $c.P
    $totalAfter += $after
    $freed = $before - $after
    Write-Host "  [done] $($c.N): freed $([math]::Round($freed/1MB,1)) MB" -ForegroundColor Green
}

Write-Host ""
$totalFreed = $totalBefore - $totalAfter
Write-Host ("Total freed: {0} MB ({1} GB)" -f [math]::Round($totalFreed/1MB,1), [math]::Round($totalFreed/1GB,2)) -ForegroundColor Cyan

Write-Host ""
Write-Host "=== RAM after clean ===" -ForegroundColor Cyan
$os = Get-CimInstance Win32_OperatingSystem
$totalGB = [math]::Round($os.TotalVisibleMemorySize/1MB,1)
$freeGB = [math]::Round($os.FreePhysicalMemory/1MB,1)
$usedGB = $totalGB - $freeGB
Write-Host ("Used: {0} GB / Free: {1} GB ({2}%)" -f $usedGB, $freeGB, [math]::Round($freeGB/$totalGB*100,1))