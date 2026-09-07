param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    $ZigPath = Join-Path $env:LOCALAPPDATA 'Microsoft\WinGet\Links\zig.exe'
    if (Test-Path -LiteralPath $ZigPath -PathType Leaf) { $Zig = Get-Item -LiteralPath $ZigPath }
    else { throw '找不到 zig，无法运行 RS485 故障端口主机回归。' }
}

$BuildDir = Join-Path $RepoRoot 'local\build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'rs485-fault-port-regression.exe'
$TestSource = Join-Path $RepoRoot 'tools\tests\Rs485FaultPortRegression.c'
$StubDir = Join-Path $RepoRoot 'tools\tests\rs485-fault-port-stubs'
$PortDir = Join-Path $RepoRoot 'shared\zephyr\port\uart'
$ZigExe = if ($Zig.Source) { $Zig.Source } else { $Zig.FullName }

& $ZigExe cc -std=c99 -Wall -Wextra -Werror "-I$StubDir" "-I$PortDir" $TestSource -o $Output
if ($LASTEXITCODE -ne 0) { throw "RS485 故障端口主机回归编译失败，退出码 $LASTEXITCODE" }
& $Output
if ($LASTEXITCODE -ne 0) { throw "RS485 故障端口主机回归失败，退出码 $LASTEXITCODE" }
