param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
. (Join-Path $PSScriptRoot 'PrepareRobotConfig.ps1')
$GeneratedConfig = Get-TestRobotConfig -Project 'HERO-M'
$Zig = Get-Command zig -ErrorAction Stop
$Output = Join-Path $RepoRoot 'local\build\host-tests\robot-mode-regression.exe'
$Args = @('cc', '-std=c11', '-Wall', '-Wextra', '-Werror', ('-I' + $GeneratedConfig))
foreach ($Dir in @('Robotconfig/HERO-M', 'shared/application/robot', 'shared/components/support')) {
    $Args += '-I' + (Join-Path $RepoRoot $Dir)
}
$Args += @((Join-Path $RepoRoot 'tools/tests/RobotModeRegression.c'), '-o', $Output)
& $Zig.Source @Args
if ($LASTEXITCODE -ne 0) { throw "运行模式主机回归编译失败：$LASTEXITCODE" }
& $Output
if ($LASTEXITCODE -ne 0) { throw "运行模式主机回归失败：$LASTEXITCODE" }
