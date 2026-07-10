param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw '找不到 zig，无法运行云台/底盘逐轴故障策略回归。'
}

$BuildDir = Join-Path $RepoRoot 'build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'motor-axis-fault-policy-regression.exe'
$Source = Join-Path $RepoRoot 'tools\tests\MotorAxisFaultPolicyRegression.c'
$MotorInclude = Join-Path $RepoRoot 'shared\application\motors'
$GimbalInclude = Join-Path $RepoRoot 'shared\application\gimbal'
$RobotInclude = Join-Path $RepoRoot 'shared\application\robot'
$SupportInclude = Join-Path $RepoRoot 'shared\components\support'

& $Zig.Source cc -std=c99 -Wall -Wextra -Werror -pedantic `
    "-I$MotorInclude" "-I$GimbalInclude" "-I$RobotInclude" "-I$SupportInclude" `
    $Source -o $Output
if ($LASTEXITCODE -ne 0) {
    throw "云台/底盘逐轴故障策略回归编译失败，退出码 $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "云台/底盘逐轴故障策略回归失败，退出码 $LASTEXITCODE"
}
