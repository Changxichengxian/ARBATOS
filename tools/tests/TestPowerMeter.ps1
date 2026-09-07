param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw '找不到 zig，无法运行 PowerMeter 主机回归。'
}

$BuildDir = Join-Path $RepoRoot 'local\build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'power-meter-regression.exe'
$TestSource = Join-Path $RepoRoot 'tools\tests\PowerMeterRegression.c'
$ModuleSource = Join-Path $RepoRoot 'shared\application\comm\can\PowerMeter.c'
$IncludeDir = Join-Path $RepoRoot 'shared\application\comm\can'

& $Zig.Source cc -std=c99 -Wall -Wextra -Werror -DPOWER_METER_HOST_TEST "-I$IncludeDir" $TestSource $ModuleSource -o $Output
if ($LASTEXITCODE -ne 0) {
    throw "PowerMeter 主机回归编译失败，退出码 $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "PowerMeter 主机回归失败，退出码 $LASTEXITCODE"
}
