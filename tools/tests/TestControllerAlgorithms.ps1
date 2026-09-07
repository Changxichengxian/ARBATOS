param()
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'PrepareRobotConfig.ps1')
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$Generated = Get-TestRobotConfig -Project HERO-M
$Zig = (Get-Command zig -ErrorAction Stop).Source
$Output = Join-Path $RepoRoot 'local\build\host-tests\controller-algorithms.exe'
$Includes = @(
    $Generated,
    (Join-Path $RepoRoot 'shared\application\robot'),
    (Join-Path $RepoRoot 'shared\controllers\differential_chassis'),
    (Join-Path $RepoRoot 'shared\controllers\speed_gimbal')
)
$CompileArgs = @('cc', '-std=c11', '-Wall', '-Wextra', '-Werror')
foreach ($Include in $Includes) { $CompileArgs += "-I$Include" }
$CompileArgs += @(
    (Join-Path $PSScriptRoot 'ControllerAlgorithmsRegression.c'),
    (Join-Path $RepoRoot 'shared\controllers\differential_chassis\DifferentialChassis.c'),
    (Join-Path $RepoRoot 'shared\controllers\speed_gimbal\SpeedGimbal.c'),
    '-o', $Output
)
& $Zig @CompileArgs
if ($LASTEXITCODE -ne 0) { throw '控制算法回归编译失败' }
& $Output
if ($LASTEXITCODE -ne 0) { throw '控制算法回归失败' }
