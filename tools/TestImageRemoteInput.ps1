param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw '找不到 zig，无法运行图传统一输入回归。'
}

$BuildDir = Join-Path $RepoRoot 'build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'image-remote-input-regression.exe'
$Source = Join-Path $RepoRoot 'tools\tests\ImageRemoteInputRegression.c'
$Args = @(
    'cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
    ('-I' + (Join-Path $RepoRoot 'tools\tests\manual-input-stubs')),
    ('-I' + (Join-Path $RepoRoot 'shared\application\input')),
    ('-I' + (Join-Path $RepoRoot 'shared\components\support')),
    $Source,
    '-o', $Output
)

& $Zig.Source @Args
if ($LASTEXITCODE -ne 0) {
    throw "图传统一输入回归编译失败，退出码 $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "图传统一输入回归失败，退出码 $LASTEXITCODE"
}
