param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw '找不到 zig，无法运行 Shoot 故障禁写策略回归。'
}

$BuildDir = Join-Path $RepoRoot 'build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'shoot-fault-policy-regression.exe'
$Source = Join-Path $RepoRoot 'tools\tests\ShootFaultPolicyRegression.c'
$IncludeDir = Join-Path $RepoRoot 'shared\application\shoot'

& $Zig.Source cc -std=c99 -Wall -Wextra -Werror -pedantic "-I$IncludeDir" $Source -o $Output
if ($LASTEXITCODE -ne 0) {
    throw "Shoot 故障禁写策略回归编译失败，退出码 $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "Shoot 故障禁写策略回归失败，退出码 $LASTEXITCODE"
}
