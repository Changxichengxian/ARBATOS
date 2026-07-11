param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw '找不到 zig，无法运行 MotorInst 许可写入主机回归。'
}

$BuildDir = Join-Path $RepoRoot 'build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'motorinst-permit-regression.exe'
$TestSource = Join-Path $RepoRoot 'tools\tests\MotorInstPermitRegression.c'
$IncludeDirs = @(
    (Join-Path $RepoRoot 'tools\tests\stubs'),
    (Join-Path $RepoRoot 'Robotconfig\SENTINEL-M'),
    (Join-Path $RepoRoot 'shared\components\support'),
    (Join-Path $RepoRoot 'shared\application\comm\can'),
    (Join-Path $RepoRoot 'shared\application\motors'),
    (Join-Path $RepoRoot 'shared\application\robot'),
    (Join-Path $RepoRoot 'shared\application\services\diagnostics')
)

$Args = @('cc', '-std=c99', '-Wall', '-Wextra', '-Werror', '-ffunction-sections', '-fdata-sections')
foreach ($Dir in $IncludeDirs) {
    $Args += "-I$Dir"
}
$Args += @($TestSource, '-Wl,--gc-sections', '-o', $Output)

& $Zig.Source @Args
if ($LASTEXITCODE -ne 0) {
    throw "MotorInst 许可写入主机回归编译失败，退出码 $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "MotorInst 许可写入主机回归失败，退出码 $LASTEXITCODE"
}
