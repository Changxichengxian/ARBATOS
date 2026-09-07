param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    $ZigPath = Join-Path $env:LOCALAPPDATA 'Microsoft\WinGet\Links\zig.exe'
    if (Test-Path -LiteralPath $ZigPath -PathType Leaf) {
        $Zig = Get-Item -LiteralPath $ZigPath
    }
    else {
        throw '找不到 zig，无法运行 CAN 故障端口主机回归。'
    }
}

$BuildDir = Join-Path $RepoRoot 'local\build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'can-fault-port-regression.exe'
$TestSource = Join-Path $RepoRoot 'tools\tests\CanFaultPortRegression.c'
$Stubs = Join-Path $RepoRoot 'tools\tests\can-fault-port-stubs'
$PortDir = Join-Path $RepoRoot 'shared\zephyr\port\can'
$ZigExe = $Zig.Source
if ([string]::IsNullOrWhiteSpace($ZigExe)) {
    $ZigExe = $Zig.FullName
}

& $ZigExe cc -std=c99 -Wall -Wextra -Werror -DCONFIG_SOC_STM32H723XX -DCONFIG_CAN_STM32H7_FDCAN "-I$Stubs" "-I$PortDir" $TestSource -o $Output
if ($LASTEXITCODE -ne 0) {
    throw "CAN 故障端口主机回归编译失败，退出码 $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "CAN 故障端口主机回归失败，退出码 $LASTEXITCODE"
}
