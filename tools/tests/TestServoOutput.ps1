param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    $ZigPath = Join-Path $env:LOCALAPPDATA 'Microsoft\WinGet\Links\zig.exe'
    if (Test-Path -LiteralPath $ZigPath) { $ZigExe = $ZigPath }
    else { throw '找不到 zig，无法运行 Servo 统一输出回归。' }
} else {
    $ZigExe = $Zig.Source
}

$BuildDir = Join-Path $RepoRoot 'local\build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'servo-output-regression.exe'
$Args = @(
    'cc', '-std=c99', '-Wall', '-Wextra', '-Werror', '-DROBOT_TASK_BUILD_SERVO=1',
    ('-I' + (Join-Path $RepoRoot 'tools\tests\stubs')),
    ('-I' + (Join-Path $RepoRoot 'shared\components\support')),
    ('-I' + (Join-Path $RepoRoot 'shared\application\robot')),
    ('-I' + (Join-Path $RepoRoot 'shared\application\services\servo')),
    (Join-Path $RepoRoot 'tools\tests\ServoOutputRegression.c'),
    '-o', $Output
)
& $ZigExe @Args
if ($LASTEXITCODE -ne 0) { throw "Servo 统一输出回归编译失败，退出码 $LASTEXITCODE" }
& $Output
if ($LASTEXITCODE -ne 0) { throw "Servo 统一输出回归失败，退出码 $LASTEXITCODE" }

$RegistryOutput = Join-Path $BuildDir 'servo-registry-regression.exe'
$CriticalHeader = Join-Path $RepoRoot 'tools\tests\ControlMgrTestCritical.h'
$RegistryArgs = @(
    'cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
    '-DCONTROL_MANAGER_TEST=1',
    '-include', $CriticalHeader,
    ('-I' + (Join-Path $RepoRoot 'tools\tests\servo_profile_stubs')),
    ('-I' + (Join-Path $RepoRoot 'shared\application\robot')),
    ('-I' + (Join-Path $RepoRoot 'shared\application\motors')),
    ('-I' + (Join-Path $RepoRoot 'shared\application\comm\can')),
    ('-I' + (Join-Path $RepoRoot 'shared\components\support')),
    (Join-Path $RepoRoot 'tools\tests\RobotControlRegistryServoRegression.c'),
    (Join-Path $RepoRoot 'shared\application\robot\ControlMgr.c'),
    '-o', $RegistryOutput
)
& $ZigExe @RegistryArgs
if ($LASTEXITCODE -ne 0) { throw "Servo 注册表回归编译失败，退出码 $LASTEXITCODE" }
& $RegistryOutput
if ($LASTEXITCODE -ne 0) { throw "Servo 注册表回归失败，退出码 $LASTEXITCODE" }
