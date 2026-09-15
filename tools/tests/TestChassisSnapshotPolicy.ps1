param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw '找不到 zig，无法运行经典底盘单帧快照策略回归。'
}

$BuildDir = Join-Path $RepoRoot 'local\build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'chassis-snapshot-policy-regression.exe'
$TestSource = Join-Path $RepoRoot 'tools\tests\ChassisSnapshotPolicyRegression.c'
$MotorHealthSource = Join-Path $RepoRoot 'shared\application\motors\MotorHealth.c'
$SnapshotSource = Join-Path $RepoRoot 'shared\application\chassis\ChassisSnapshotHelpers.inc'
$SnapshotText = [System.IO.File]::ReadAllText($SnapshotSource, [System.Text.Encoding]::UTF8)
# 直接编译生产采集段，测试只替代反馈来源和时钟，避免另写一份采集顺序。
$BatchMatch = [regex]::Match($SnapshotText,
    '(?ms)^    ChassisMotorReadPlanBuild\(axisId, configured, bound, &readPlan\);.*?(?=^    if \(control != NULL\))')
$UnavailableMatch = [regex]::Match($SnapshotText,
    '(?ms)^static void ChassisMotorHealthUnavailable\(.*?(?=^static uint32_t ChassisMotorFeedbackTimeoutMs\()')
if (-not $BatchMatch.Success -or -not $UnavailableMatch.Success) {
    throw '生产快照采集段边界已变化，请更新回归测试抽取入口。'
}
$BatchInclude = Join-Path $BuildDir 'ChassisSnapshotBatchUnderTest.inc'
$UnavailableInclude = Join-Path $BuildDir 'ChassisSnapshotUnavailableUnderTest.inc'
$Utf8 = New-Object System.Text.UTF8Encoding($false)
[System.IO.File]::WriteAllText($BatchInclude, $BatchMatch.Value, $Utf8)
[System.IO.File]::WriteAllText($UnavailableInclude, $UnavailableMatch.Value, $Utf8)
$IncludeDirs = @(
    $BuildDir,
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
