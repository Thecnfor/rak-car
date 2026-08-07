# run_task3.ps1 - Task3 识别+射击流水线启动脚本（含 ERNIE token 接入）
# 用法:
#   .\run_task3.ps1 -Token "你的token"                 # 直接传 token
#   .\run_task3.ps1                                    # 交互输入 token
#   $env:ERNIE_ACCESS_TOKEN="你的token"; .\run_task3.ps1
# 可选: -NoPause（识别完直接射击，跳过人工确认）, -ExtraArgs "--creep-speed 0.18"
param(
    [string]$Token = $env:ERNIE_ACCESS_TOKEN,
    [switch]$NoPause,
    [string]$ExtraArgs = ''
)
$ErrorActionPreference = 'Stop'

# 仓库根 = main/task/task3 -> 上三级
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..\..')).Path

# token 接入: 参数 > 环境变量 > 交互输入
if ([string]::IsNullOrWhiteSpace($Token)) {
    $Token = (Read-Host '请输入 ERNIE access token').Trim()
}
if ([string]::IsNullOrWhiteSpace($Token)) {
    Write-Host '[错误] 未提供 ERNIE token' -ForegroundColor Red
    Write-Host '用法: .\run_task3.ps1 -Token "<token>"' -ForegroundColor Yellow
    exit 2
}

$env:ERNIE_ACCESS_TOKEN = $Token
$env:PYTHONIOENCODING = 'utf-8'
Set-Location $repo

$argsList = New-Object System.Collections.Generic.List[string]
$argsList.Add('-m')
$argsList.Add('main.task.task3.task3_pipeline')
if ($NoPause) { $argsList.Add('--no-pause') }
if (-not [string]::IsNullOrWhiteSpace($ExtraArgs)) {
    foreach ($a in ($ExtraArgs -split '\s+')) {
        if ($a) { $argsList.Add($a) }
    }
}
Write-Host "[task3] python $($argsList -join ' ')" -ForegroundColor Cyan

# Python 正常会往 stderr 打 [warn] 等日志；不能用 Stop 模式，否则被误判为致命错误
$prevEAP = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
python @($argsList)
$code = $LASTEXITCODE
$ErrorActionPreference = $prevEAP
exit $code