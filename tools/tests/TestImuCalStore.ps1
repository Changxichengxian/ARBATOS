param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    $ZigPath = 'C:\Users\28111\AppData\Local\Microsoft\WinGet\Links\zig.exe'
    if (Test-Path -LiteralPath $ZigPath) { $ZigExe = $ZigPath }
    else { throw '找不到 zig，无法运行 ImuCalStore 主机回归。' }
} else { $ZigExe = $Zig.Source }

$BuildDir = Join-Path $RepoRoot 'local\build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Source = Join-Path $RepoRoot 'tools\tests\ImuCalStoreRegression.c'
$Stubs = Join-Path $RepoRoot 'tools\tests\imu-store-stubs'
$Sensors = Join-Path $RepoRoot 'shared\zephyr\port\sensors'

foreach ($Mount in 1..3) {
    $Output = Join-Path $BuildDir "imu-cal-store-regression-$Mount.exe"
    $Args = @('cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-DCONFIG_USE_DT_CODE_PARTITION=1', '-DCONFIG_FLASH_LOAD_OFFSET=0', '-DCONFIG_FLASH_LOAD_SIZE=0xc0000', '-DCONFIG_ARBATOS_PREFLIGHT_ONLY=1', '-DTEST_PREFLIGHT=1',
              "-DTEST_IMU_MOUNT=$Mount", "-I$Stubs", "-I$RepoRoot\tools\tests\imu-frame-stubs", "-I$Sensors",
              $Source, '-lm', '-o', $Output)
    & $ZigExe @Args
    if ($LASTEXITCODE -ne 0) { throw "ImuCalStore 主机回归编译失败（安装方向 $Mount），退出码 $LASTEXITCODE" }
    & $Output
    if ($LASTEXITCODE -ne 0) { throw "ImuCalStore 主机回归失败（安装方向 $Mount），退出码 $LASTEXITCODE" }
}

$FormalOutput = Join-Path $BuildDir 'imu-cal-store-formal-regression.exe'
$FormalArgs = @('cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-DCONFIG_USE_DT_CODE_PARTITION=1', '-DCONFIG_FLASH_LOAD_OFFSET=0', '-DCONFIG_FLASH_LOAD_SIZE=0xc0000', '-DTEST_IMU_MOUNT=1',
                "-I$Stubs", "-I$RepoRoot\tools\tests\imu-frame-stubs", "-I$Sensors", $Source, '-lm', '-o', $FormalOutput)
& $ZigExe @FormalArgs
if ($LASTEXITCODE -ne 0) { throw "ImuCalStore 正式固件授权回归编译失败，退出码 $LASTEXITCODE" }
& $FormalOutput
if ($LASTEXITCODE -ne 0) { throw "ImuCalStore 正式固件授权回归失败，退出码 $LASTEXITCODE" }
