$paths = @(
    @{ N = "UserTemp";        P = "$env:TEMP";                                                                  R = "low" },
    @{ N = "WinTemp";         P = "C:\Windows\Temp";                                                            R = "low" },
    @{ N = "EdgeCache";       P = "C:\Users\$env:USERNAME\AppData\Local\Microsoft\Edge\User Data\Default\Cache"; R = "very-low" },
    @{ N = "EdgeCodeCache";   P = "C:\Users\$env:USERNAME\AppData\Local\Microsoft\Edge\User Data\Default\Code Cache"; R = "very-low" },
    @{ N = "ChromeCache";     P = "C:\Users\$env:USERNAME\AppData\Local\Google\Chrome\User Data\Default\Cache"; R = "very-low" },
    @{ N = "WeChatCache";     P = "C:\Users\$env:USERNAME\AppData\Roaming\Tencent\WeChat";                     R = "very-low" },
    @{ N = "xwechat_files";   P = "C:\Users\$env:USERNAME\xwechat_files";                                        R = "mid" },
    @{ N = "pipCache";        P = "C:\Users\$env:USERNAME\AppData\Local\pip\cache";                            R = "very-low" },
    @{ N = "npmCache";        P = "C:\Users\$env:USERNAME\AppData\Local\npm-cache";                             R = "very-low" },
    @{ N = "condaCache";      P = "C:\Users\$env:USERNAME\.conda\pkgs";                                          R = "very-low" },
    @{ N = "traeCache";       P = "C:\Users\$env:USERNAME\.trae-cn";                                            R = "mid" }
)

function Get-DirSize($p) {
    if (-not (Test-Path $p)) { return $null }
    $s = (Get-ChildItem $p -Recurse -Force -ErrorAction SilentlyContinue |
          Where-Object { -not $_.PSIsContainer } |
          Measure-Object -Property Length -Sum).Sum
    if ($null -eq $s) { $s = 0 }
    return [math]::Round($s/1GB, 2)
}

Write-Host "=== Cleanable candidates ===" -ForegroundColor Cyan
$total = 0
foreach ($c in $paths) {
    $gb = Get-DirSize $c.P
    if ($null -ne $gb) {
        $color = switch ($c.R) {
            "very-low" { "Green" }
            "low"      { "Yellow" }
            default    { "Red" }
        }
        Write-Host ("[{0,-8}] {1,-20} {2,8} GB  {3}" -f $c.R, $c.N, $gb, $c.P) -ForegroundColor $color
        if ($c.R -eq "very-low") { $total += $gb }
    }
}
Write-Host ""
Write-Host ("Total very-low risk available: {0} GB" -f $total) -ForegroundColor Green