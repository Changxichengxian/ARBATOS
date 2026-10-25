param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw '找不到 zig，无法运行 IMU 服务主机回归。此测试不属于常规 CheckAll 的强制依赖。'
}

$BuildDir = Join-Path $RepoRoot 'local\build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'imu-service-regression.exe'
$TestSource = Join-Path $RepoRoot 'tools\tests\ImuServiceRegression.c'
$IncludeDirs = @(
    (Join-Path $RepoRoot 'tools\tests\imu-service-stubs'),
    (Join-Path $RepoRoot 'shared\zephyr\port\sensors')
)

$Args = @('cc', '-std=c99', '-Wall', '-Wextra', '-Werror')
foreach ($Dir in $IncludeDirs) {
    $Args += "-I$Dir"
}
$Args += @($TestSource, '-o', $Output)

& $Zig.Source @Args
if ($LASTEXITCODE -ne 0) {
    throw "IMU 服务主机回归编译失败，退出码 $LASTEXITCODE"
}
& $Output
if ($LASTEXITCODE -ne 0) {
    throw "IMU 服务主机回归失败，退出码 $LASTEXITCODE"
}
