param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw '找不到 zig，无法运行 Shoot 控制域生命周期回归。'
}

$BuildDir = Join-Path $RepoRoot 'build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'shoot-ctrl-regression.exe'
$Sources = @(
    (Join-Path $RepoRoot 'tools\tests\ShootCtrlRegression.c'),
    (Join-Path $RepoRoot 'shared\application\robot\ControlMgr.c'),
    (Join-Path $RepoRoot 'shared\application\shoot\ShootCtrl.c')
)
$IncludeDirs = @(
    (Join-Path $RepoRoot 'tools\tests\stubs'),
    (Join-Path $RepoRoot 'shared\application\robot'),
    (Join-Path $RepoRoot 'shared\application\shoot')
)

$Args = @('cc', '-std=c99', '-Wall', '-Wextra', '-Werror', '-DROBOT_TASK_BUILD_SHOOT_RM=1')
foreach ($Dir in $IncludeDirs) {
    $Args += "-I$Dir"
}
$Args += $Sources
$Args += @('-o', $Output)

& $Zig.Source @Args
if ($LASTEXITCODE -ne 0) {
    throw "Shoot 控制域生命周期回归编译失败，退出码 $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "Shoot 控制域生命周期回归失败，退出码 $LASTEXITCODE"
}
