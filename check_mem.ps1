Write-Host "=== 物理内存 ===" -ForegroundColor Cyan
$os = Get-CimInstance Win32_OperatingSystem
$totalMem = [math]::Round($os.TotalVisibleMemorySize/1MB,1)
$freeMem = [math]::Round($os.FreePhysicalMemory/1MB,1)
$usedMem = $totalMem - $freeMem
$usedPct = [math]::Round(($usedMem/$totalMem)*100,1)
Write-Host "总: $totalMem GB / 已用: $usedMem GB / 剩余: $freeMem GB ($([math]::Round($freeMem/$totalMem*100,1))%)"
Write-Host ""
Write-Host "=== Top 20 内存占用进程 ===" -ForegroundColor Cyan
Get-Process | Sort-Object -Property WorkingSet64 -Descending |
    Select-Object -First 20 `
        @{N='ProcessName';E={$_.ProcessName}},
        @{N='PID';E={$_.Id}},
        @{N='MemGB';E={[math]::Round($_.WorkingSet64/1GB,2)}},
        @{N='CPU%';E={[math]::Round($_.CPU,1)}},
        @{N='Threads';E={$_.Threads.Count}} |
    Format-Table -AutoSize
Write-Host "=== 虚拟内存 (页面文件) ===" -ForegroundColor Cyan
$pf = Get-CimInstance Win32_PageFileUsage
foreach ($p in $pf) {
    $allocGB = [math]::Round($p.AllocatedBaseSize/1024,1)
    $peakGB = [math]::Round($p.PeakUsage/1024,1)
    Write-Host "  $($p.Name): 分配 $allocGB GB / 峰值使用 $peakGB GB"
}
Write-Host ""
Write-Host "=== 内存压力 (关键指标) ===" -ForegroundColor Yellow
$cache = Get-CimInstance Win32_ComputerSystem
$perf = (Get-Counter '\Memory\Cache Bytes','\Memory\Pool Nonpaged Bytes','\Memory\Pool Paged Bytes' -ErrorAction SilentlyContinue).CounterSamples
foreach ($s in $perf) {
    $gb = [math]::Round($s.CookedValue/1GB,2)
    Write-Host "  $($s.Path.Split('\')[-1]): $gb GB"
}