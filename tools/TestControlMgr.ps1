param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw '找不到 zig，无法运行控制管理器回归。'
}

$BuildDir = Join-Path $RepoRoot 'build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'control-mgr-regression.exe'
$CriticalHeader = Join-Path $RepoRoot 'tools\tests\ControlMgrTestCritical.h'
$Sources = @(
    (Join-Path $RepoRoot 'tools\tests\ControlMgrRegression.c'),
    (Join-Path $RepoRoot 'shared\application\robot\ControlMgr.c')
)
$Args = @(
    'cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
    '-DCONTROL_MANAGER_TEST=1',
    '-include', $CriticalHeader,
    ('-I' + (Join-Path $RepoRoot 'shared\application\robot'))
)
$Args += $Sources
$Args += @('-o', $Output)

& $Zig.Source @Args
if ($LASTEXITCODE -ne 0) {
    throw "控制管理器回归编译失败，退出码 $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "控制管理器回归失败，退出码 $LASTEXITCODE"
}

$ActuatorPolicySource = Join-Path $RepoRoot 'tools\tests\ControlActuatorPolicyRegression.c'
$ActuatorPolicyOutput = Join-Path $BuildDir 'control-actuator-policy-regression.exe'
$ActuatorPolicyArgs = @(
    'cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
    ('-I' + (Join-Path $RepoRoot 'shared\application\robot')),
    ('-I' + (Join-Path $RepoRoot 'shared\components\support')),
    $ActuatorPolicySource,
    '-o', $ActuatorPolicyOutput
)

& $Zig.Source @ActuatorPolicyArgs
if ($LASTEXITCODE -ne 0) {
    throw "执行器所有权策略回归编译失败，退出码 $LASTEXITCODE"
}

& $ActuatorPolicyOutput
if ($LASTEXITCODE -ne 0) {
    throw "执行器所有权策略回归失败，退出码 $LASTEXITCODE"
}

$AbiSource = Join-Path $RepoRoot 'tools\tests\ControlMgrAbiRegression.c'
$AbiOutput = Join-Path $BuildDir 'control-mgr-abi-regression.obj'
$AbiArgs = @(
    'cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-c', $AbiSource,
    ('-I' + (Join-Path $RepoRoot 'shared\application\services\diagnostics')),
    ('-I' + (Join-Path $RepoRoot 'shared\application\robot')),
    ('-I' + (Join-Path $RepoRoot 'shared\components\support')),
    ('-I' + (Join-Path $RepoRoot 'Robotconfig\HERO-M')),
    '-o', $AbiOutput
)

& $Zig.Source @AbiArgs
if ($LASTEXITCODE -ne 0) {
    throw "控制管理器 ABI 回归编译失败，退出码 $LASTEXITCODE"
}

$ArmAbiOutput = Join-Path $BuildDir 'control-mgr-abi-arm32-regression.obj'
$ArmAbiArgs = @(
    'cc', '-target', 'arm-freestanding-eabi', '-mcpu=cortex_m7', '-mfloat-abi=hard',
    '-std=c11', '-Wall', '-Wextra', '-Werror', '-c', $AbiSource,
    ('-I' + (Join-Path $RepoRoot 'shared\application\services\diagnostics')),
    ('-I' + (Join-Path $RepoRoot 'shared\application\robot')),
    ('-I' + (Join-Path $RepoRoot 'shared\components\support')),
    ('-I' + (Join-Path $RepoRoot 'Robotconfig\HERO-M')),
    '-o', $ArmAbiOutput
)

& $Zig.Source @ArmAbiArgs
if ($LASTEXITCODE -ne 0) {
    throw "控制管理器 ARM32 ABI 回归编译失败，退出码 $LASTEXITCODE"
}
