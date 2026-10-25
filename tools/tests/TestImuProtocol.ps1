param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw '找不到 zig，无法运行 IMU 协议与方向主机回归。此测试不属于常规 CheckAll 的强制依赖。'
}

$BuildDir = Join-Path $RepoRoot 'local\build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$TestSource = Join-Path $RepoRoot 'tools\tests\ImuProtocolRegression.c'
$IncludeDirs = @(
    (Join-Path $RepoRoot 'tools\tests\imu-frame-stubs'),
    (Join-Path $RepoRoot 'shared\zephyr\port\sensors')
)

foreach ($Mount in 1..3) {
    $Output = Join-Path $BuildDir "imu-protocol-regression-$Mount.exe"
    $Args = @('cc', '-std=c99', '-Wall', '-Wextra', '-Werror', "-DTEST_IMU_MOUNT=$Mount")
    foreach ($Dir in $IncludeDirs) {
        $Args += "-I$Dir"
    }
    $Args += @($TestSource, '-o', $Output)

    & $Zig.Source @Args
    if ($LASTEXITCODE -ne 0) {
        throw "IMU 协议与方向主机回归编译失败（方向 $Mount），退出码 $LASTEXITCODE"
    }
    & $Output
    if ($LASTEXITCODE -ne 0) {
        throw "IMU 协议与方向主机回归失败（方向 $Mount），退出码 $LASTEXITCODE"
    }
}
