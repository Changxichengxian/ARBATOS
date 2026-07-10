param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw '找不到 zig，无法运行 RobotLifecycle 主机回归。'
}

$BuildDir = Join-Path $RepoRoot 'build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'robot-lifecycle-regression.exe'
$Sources = @(
    (Join-Path $RepoRoot 'tools\tests\RobotLifecycleRegression.c'),
    (Join-Path $RepoRoot 'shared\application\robot\RobotLifecycle.c')
)
$IncludeDirs = @(
    (Join-Path $RepoRoot 'tools\tests\robot-lifecycle-stubs'),
    (Join-Path $RepoRoot 'shared\application\robot')
)

$Args = @('cc', '-std=c11', '-Wall', '-Wextra', '-Werror')
foreach ($Dir in $IncludeDirs) {
    $Args += "-I$Dir"
}
$Args += $Sources
$Args += @('-o', $Output)

& $Zig.Source @Args
if ($LASTEXITCODE -ne 0) {
    throw "RobotLifecycle 主机回归编译失败，退出码 $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "RobotLifecycle 主机回归失败，退出码 $LASTEXITCODE"
}
