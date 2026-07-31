$c = Get-Volume -DriveLetter C
$usedGB = [math]::Round(($c.Size - $c.SizeRemaining)/1GB,1)
$freeGB = [math]::Round($c.SizeRemaining/1GB,1)
$totalGB = [math]::Round($c.Size/1GB,1)
$freePct = [math]::Round(($c.SizeRemaining/$c.Size)*100,1)
Write-Host "C 盘: 总 $totalGB GB / 已用 $usedGB GB / 剩余 $freeGB GB ($freePct%)" -ForegroundColor Yellow
Write-Host ""
Write-Host "=== 用户目录大文件夹 (Top 10) ===" -ForegroundColor Cyan
$skip = @('$Recycle.Bin','$WinREAgent','$SysReset','PerfLogs','Program Files','Program Files (x86)','ProgramData','Users','Windows')
Get-ChildItem C:\ -Directory -Force -ErrorAction SilentlyContinue |
    Where-Object { $_.Name -notin $skip } |
    ForEach-Object {
        $size = (Get-ChildItem $_.FullName -Recurse -Force -ErrorAction SilentlyContinue | Measure-Object -Property Length -Sum).Sum
        [PSCustomObject]@{Name=$_.Name; GB=[math]::Round($size/1GB,2)}
    } | Sort-Object GB -Descending | Select-Object -First 10 | Format-Table -AutoSize
Write-Host "=== C:\Users\$env:USERNAME 下大文件夹 (Top 10) ===" -ForegroundColor Cyan
$userDir = "C:\Users\$env:USERNAME"
Get-ChildItem $userDir -Directory -Force -ErrorAction SilentlyContinue |
    ForEach-Object {
        $size = (Get-ChildItem $_.FullName -Recurse -Force -ErrorAction SilentlyContinue | Measure-Object -Property Length -Sum).Sum
        if ($size -gt 100MB) {
            [PSCustomObject]@{Name=$_.Name; GB=[math]::Round($size/1GB,2)}
        }
    } | Sort-Object GB -Descending | Select-Object -First 10 | Format-Table -AutoSize
Write-Host "=== 回收站大小 ===" -ForegroundColor Cyan
$recycle = (Get-ChildItem 'C:\$Recycle.Bin' -Force -Recurse -ErrorAction SilentlyContinue | Measure-Object -Property Length -Sum).Sum
Write-Host "回收站: $([math]::Round($recycle/1GB,2)) GB"
Write-Host "=== Windows 临时目录 ===" -ForegroundColor Cyan
$temp = "C:\Users\$env:USERNAME\AppData\Local\Temp"
$tempSize = (Get-ChildItem $temp -Recurse -Force -ErrorAction SilentlyContinue | Measure-Object -Property Length -Sum).Sum
Write-Host "用户 Temp: $([math]::Round($tempSize/1GB,2)) GB"
$winTemp = "C:\Windows\Temp"
$winTempSize = (Get-ChildItem $winTemp -Recurse -Force -ErrorAction SilentlyContinue | Measure-Object -Property Length -Sum).Sum
Write-Host "Windows Temp: $([math]::Round($winTempSize/1GB,2)) GB"