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
        throw '找不到 zig，无法运行 RobotFaultZephyr 主机回归。'
    }
}

$BuildDir = Join-Path $RepoRoot 'local\build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'robot-fault-zephyr-regression.exe'
$TestSource = Join-Path $RepoRoot 'tools\tests\RobotFaultZephyrRegression.c'
$Stubs = Join-Path $RepoRoot 'tools\tests\robot-fault-zephyr-stubs'
$PlatformDir = Join-Path $RepoRoot 'shared\zephyr\port\platform'
$HalDir = Join-Path $RepoRoot 'shared\hal'
$ZigExe = if ([string]::IsNullOrWhiteSpace($Zig.Source)) { $Zig.FullName } else { $Zig.Source }

& $ZigExe cc -std=c99 -Wall -Wextra -Werror -DCONFIG_SOC_STM32H723XX "-I$Stubs" "-I$PlatformDir" "-I$HalDir" $TestSource -o $Output
if ($LASTEXITCODE -ne 0) {
    throw "RobotFaultZephyr 主机回归编译失败，退出码 $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "RobotFaultZephyr 主机回归失败，退出码 $LASTEXITCODE"
}
