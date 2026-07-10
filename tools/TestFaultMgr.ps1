param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw '找不到 zig，无法运行 FaultMgr 主机回归。'
}

$BuildDir = Join-Path $RepoRoot 'build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'fault-mgr-regression.exe'
$Source = Join-Path $RepoRoot 'shared\application\robot\FaultMgr.c'
$TestSource = Join-Path $RepoRoot 'tools\tests\FaultMgrRegression.c'
$IncludeDir = Join-Path $RepoRoot 'shared\application\robot'

$Args = @(
    'cc',
    '-std=c99',
    '-Wall',
    '-Wextra',
    '-Werror',
    '-pedantic',
    "-I$IncludeDir",
    $Source,
    $TestSource,
    '-o',
    $Output
)

& $Zig.Source @Args
if ($LASTEXITCODE -ne 0) {
    throw "FaultMgr 主机回归编译失败，退出码 $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "FaultMgr 主机回归失败，退出码 $LASTEXITCODE"
}
