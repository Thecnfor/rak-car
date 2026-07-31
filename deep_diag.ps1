Write-Host "=== Physical CPU info ===" -ForegroundColor Cyan
$cpu = Get-CimInstance Win32_Processor | Select-Object -First 1
Write-Host ("Name: {0}" -f $cpu.Name)
Write-Host ("Cores: {0}  Logical: {1}" -f $cpu.NumberOfCores, $cpu.NumberOfLogicalProcessors)
Write-Host ("MaxClock: {0} GHz  CurrentClock: {1} GHz" -f [math]::Round($cpu.MaxClockSpeed/1000,2), [math]::Round($cpu.CurrentClockSpeed/1000,2))

Write-Host ""
Write-Host "=== CPU usage overall (5-sec sample) ===" -ForegroundColor Cyan
$cpuLoad = (Get-Counter '\Processor(_Total)\% Processor Time' -SampleInterval 2 -MaxSamples 3 -ErrorAction SilentlyContinue).CounterSamples
foreach ($s in $cpuLoad) {
    Write-Host ("  {0}: {1}%" -f $s.Path.Split('\')[-1], [math]::Round($s.CookedValue,1))
}

Write-Host ""
Write-Host "=== Top 15 by CPU time (累计 CPU 时间, 不一定是当前负载) ===" -ForegroundColor Cyan
Get-Process | Sort-Object -Property CPU -Descending |
    Select-Object -First 15 `
        @{N='Process';E={$_.ProcessName}},
        @{N='PID';E={$_.Id}},
        @{N='CPUTotal_s';E={[math]::Round($_.CPU,1)}},
        @{N='MemGB';E={[math]::Round($_.WorkingSet64/1GB,2)}},
        @{N='Threads';E={$_.Threads.Count}},
        @{N='Handles';E={$_.HandleCount}} |
    Format-Table -AutoSize

Write-Host "=== Top 15 by handle count (句柄 = 系统资源占用) ===" -ForegroundColor Cyan
Get-Process | Sort-Object -Property HandleCount -Descending |
    Select-Object -First 15 `
        @{N='Process';E={$_.ProcessName}},
        @{N='PID';E={$_.Id}},
        @{N='Handles';E={$_.HandleCount}},
        @{N='Threads';E={$_.Threads.Count}},
        @{N='MemGB';E={[math]::Round($_.WorkingSet64/1GB,2)}} |
    Format-Table -AutoSize

Write-Host "=== Disk activity ===" -ForegroundColor Cyan
$disks = Get-Counter '\PhysicalDisk(_Total)\% Disk Time','\PhysicalDisk(_Total)\Avg. Disk Queue Length','\PhysicalDisk(_Total)\Disk Reads/sec','\PhysicalDisk(_Total)\Disk Writes/sec' -SampleInterval 2 -MaxSamples 2 -ErrorAction SilentlyContinue
foreach ($s in $disks.CounterSamples) {
    $name = $s.Path.Split('\')[-1]
    Write-Host ("  {0}: {1}" -f $name, [math]::Round($s.CookedValue,2))
}

Write-Host ""
Write-Host "=== Page faults / sec (内存压力指标) ===" -ForegroundColor Cyan
$pf = Get-Counter '\Memory\Page Faults/sec','\Memory\Pages Input/sec','\Memory\Pages Output/sec' -ErrorAction SilentlyContinue
foreach ($s in $pf.CounterSamples) {
    $name = $s.Path.Split('\')[-1]
    Write-Host ("  {0}: {1}" -f $name, [math]::Round($s.CookedValue,1))
}

Write-Host ""
Write-Host "=== System uptime ===" -ForegroundColor Cyan
$uptime = (Get-Date) - $os.LastBootUpTime
Write-Host ("Up: {0} days {1} hours" -f $uptime.Days, $uptime.Hours)