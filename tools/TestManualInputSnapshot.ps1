param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw '找不到 zig，无法运行统一手动输入快照回归。'
}

$BuildDir = Join-Path $RepoRoot 'build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'manual-input-snapshot-regression.exe'
$Source = Join-Path $RepoRoot 'tools\tests\ManualInputSnapshotRegression.c'
$Args = @(
    'cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
    ('-I' + (Join-Path $RepoRoot 'tools\tests\manual-input-stubs')),
    ('-I' + (Join-Path $RepoRoot 'shared\application\input')),
    $Source,
    '-o', $Output
)

& $Zig.Source @Args
if ($LASTEXITCODE -ne 0) {
    throw "统一手动输入快照回归编译失败，退出码 $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "统一手动输入快照回归失败，退出码 $LASTEXITCODE"
}
