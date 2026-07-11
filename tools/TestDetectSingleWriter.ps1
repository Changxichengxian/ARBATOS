param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw '找不到 zig，无法运行 Detect 单写者主机回归。'
}

$BuildDir = Join-Path $RepoRoot 'build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'detect-single-writer-regression.exe'
$Source = Join-Path $RepoRoot 'tools\tests\DetectSingleWriterRegression.c'
$Args = @(
    'cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
    ('-I' + (Join-Path $RepoRoot 'Robotconfig\CARRIER-A')),
    ('-I' + (Join-Path $RepoRoot 'shared\application\services\diagnostics')),
    ('-I' + (Join-Path $RepoRoot 'shared\application\robot')),
    ('-I' + (Join-Path $RepoRoot 'shared\components\support')),
    $Source,
    '-o', $Output
)

& $Zig.Source @Args
if ($LASTEXITCODE -ne 0) {
    throw "Detect 单写者主机回归编译失败，退出码 $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "Detect 单写者主机回归失败，退出码 $LASTEXITCODE"
}
