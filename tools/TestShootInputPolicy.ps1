param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw '找不到 zig，无法运行 Shoot 输入安全恢复门控回归。'
}

$BuildDir = Join-Path $RepoRoot 'build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'shoot-input-policy-regression.exe'
$Source = Join-Path $RepoRoot 'tools\tests\ShootInputPolicyRegression.c'
$IncludeDir = Join-Path $RepoRoot 'shared\application\shoot'
$Args = @(
    'cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
    ("-I" + $IncludeDir), $Source, '-o', $Output
)

& $Zig.Source @Args
if ($LASTEXITCODE -ne 0) {
    throw "Shoot 输入安全恢复门控回归编译失败，退出码 $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "Shoot 输入安全恢复门控回归失败，退出码 $LASTEXITCODE"
}
