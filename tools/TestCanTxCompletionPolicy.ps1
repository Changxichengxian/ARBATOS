param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw '找不到 zig，无法运行 CAN 物理完成策略回归。'
}

$BuildDir = Join-Path $RepoRoot 'build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'can-tx-completion-policy-regression.exe'
$TestSource = Join-Path $RepoRoot 'tools\tests\CanTxCompletionPolicyRegression.c'
$IncludeDirs = @(
    (Join-Path $RepoRoot 'tools\tests\stubs'),
    (Join-Path $RepoRoot 'shared\components\support'),
    (Join-Path $RepoRoot 'shared\application\robot'),
    (Join-Path $RepoRoot 'shared\application\comm\can'),
    (Join-Path $RepoRoot 'shared\hal')
)

$Args = @('cc', '-std=c99', '-Wall', '-Wextra', '-Werror')
foreach ($Dir in $IncludeDirs) {
    $Args += "-I$Dir"
}
$Args += @($TestSource, '-o', $Output)

& $Zig.Source @Args
if ($LASTEXITCODE -ne 0) {
    throw "CAN 物理完成策略回归编译失败，退出码 $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "CAN 物理完成策略回归失败，退出码 $LASTEXITCODE"
}
