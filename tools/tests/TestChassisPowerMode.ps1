param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
. (Join-Path $PSScriptRoot 'PrepareRobotConfig.ps1')
$GeneratedConfig = Get-TestRobotConfig -Project 'HERO-M'
$Zig = Get-Command zig -ErrorAction Stop
$BuildDir = Join-Path $RepoRoot 'local\build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'chassis-power-mode-regression.exe'
$Includes = @('shared/application/motors', 'shared/application/robot',
    'shared/components/support', 'shared/components/controller',
    'shared/application/comm/referee', 'shared/application/services/diagnostics',
    'shared/zephyr/port/algorithm/include', 'Robotconfig/HERO-M')
$IncludeArgs = @($Includes | ForEach-Object { '-I' + (Join-Path $RepoRoot $_) })
& $Zig.Source cc -std=c99 -Wall -Wextra -Werror $IncludeArgs "-I$GeneratedConfig" `
    (Join-Path $RepoRoot 'tools/tests/ChassisPowerModeRegression.c') `
    (Join-Path $RepoRoot 'shared/components/controller/ChassisPowerLimiter.c') `
    (Join-Path $RepoRoot 'shared/application/motors/MotorModelDb.c') -o $Output
if ($LASTEXITCODE -ne 0) { throw '功率策略回归编译失败' }
& $Output
if ($LASTEXITCODE -ne 0) { throw '功率策略回归失败' }
