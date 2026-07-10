param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw '找不到 zig，无法运行经典底盘单帧快照策略回归。'
}

$BuildDir = Join-Path $RepoRoot 'build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'chassis-snapshot-policy-regression.exe'
$TestSource = Join-Path $RepoRoot 'tools\tests\ChassisSnapshotPolicyRegression.c'
$MotorHealthSource = Join-Path $RepoRoot 'shared\application\motors\MotorHealth.c'
$IncludeDirs = @(
    (Join-Path $RepoRoot 'shared\components\support'),
    (Join-Path $RepoRoot 'shared\components\controller'),
    (Join-Path $RepoRoot 'shared\application\robot'),
    (Join-Path $RepoRoot 'shared\application\motors'),
    (Join-Path $RepoRoot 'shared\application\gimbal'),
    (Join-Path $RepoRoot 'shared\application\comm\can'),
    (Join-Path $RepoRoot 'shared\application\chassis')
)

$Args = @('cc', '-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic')
foreach ($Dir in $IncludeDirs) {
    $Args += "-I$Dir"
}
$Args += @($TestSource, $MotorHealthSource, '-o', $Output)

& $Zig.Source @Args
if ($LASTEXITCODE -ne 0) {
    throw "经典底盘单帧快照策略回归编译失败，退出码 $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "经典底盘单帧快照策略回归失败，退出码 $LASTEXITCODE"
}
