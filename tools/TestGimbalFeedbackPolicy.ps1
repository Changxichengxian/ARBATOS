param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw '找不到 zig，无法运行云台反馈降级策略回归。'
}

$BuildDir = Join-Path $RepoRoot 'build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'gimbal-feedback-policy-regression.exe'
$Source = Join-Path $RepoRoot 'tools\tests\GimbalFeedbackPolicyRegression.c'
$GimbalInclude = Join-Path $RepoRoot 'shared\application\gimbal'
$RobotInclude = Join-Path $RepoRoot 'shared\application\robot'
$SupportInclude = Join-Path $RepoRoot 'shared\components\support'

& $Zig.Source cc -std=c99 -Wall -Wextra -Werror -pedantic `
    "-I$GimbalInclude" "-I$RobotInclude" "-I$SupportInclude" `
    $Source -lm -o $Output
if ($LASTEXITCODE -ne 0) {
    throw "云台反馈降级策略回归编译失败，退出码 $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "云台反馈降级策略回归失败，退出码 $LASTEXITCODE"
}
