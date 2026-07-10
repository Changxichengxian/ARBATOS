param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw '找不到 zig，无法运行 Unitree 主机回归。'
}

$BuildDir = Join-Path $RepoRoot 'build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'unitree-motor-policy-regression.exe'
$TestSource = Join-Path $RepoRoot 'tools\tests\UnitreeMotorPolicyRegression.c'
$IncludeDirs = @(
    (Join-Path $RepoRoot 'shared\components\support'),
    (Join-Path $RepoRoot 'shared\application\robot'),
    (Join-Path $RepoRoot 'shared\application\motors')
)

$Args = @('cc', '-std=c99', '-Wall', '-Wextra', '-Werror')
foreach ($Dir in $IncludeDirs) {
    $Args += "-I$Dir"
}
$Args += @($TestSource, '-o', $Output)

& $Zig.Source @Args
if ($LASTEXITCODE -ne 0) {
    throw "Unitree 主机回归编译失败，退出码 $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "Unitree 主机回归失败，退出码 $LASTEXITCODE"
}

$ArmSource = Get-Content -LiteralPath (Join-Path $RepoRoot 'shared\application\arm\ArmMotion.c') -Raw -Encoding UTF8
foreach ($Forbidden in @('UnitreeMotorSendCmd\s*\(',
                         'UnitreeMotorConfigure\s*\(',
                         'UnitreeMotorRefresh\s*\(',
                         'ArmUpdateJ0LowStateFromUnitree')) {
    if ($ArmSource -match $Forbidden) {
        throw "ArmMotion 仍持有 Unitree 发送/驱动写职责：$Forbidden"
    }
}
if ($ArmSource -notmatch 'UnitreeMotorGetStateCopy\s*\(') {
    throw 'ArmMotion 未通过一致副本读取 Unitree 诊断状态。'
}

$SendSites = @(rg -l 'UnitreeMotorSendActuator\s*\(' `
    (Join-Path $RepoRoot 'shared\application') -g '*.c')
$ExpectedSendSites = @(
    (Join-Path $RepoRoot 'shared\application\motors\N6014bMotorDriver.c'),
    (Join-Path $RepoRoot 'shared\application\motors\UnitreeMotorDriver.c')
)
if ($SendSites.Count -ne 2) {
    throw "UnitreeMotorSendActuator 调用边界异常：期望定义+唯一 CanTx 分发点，实际文件数 $($SendSites.Count)。"
}
foreach ($Expected in $ExpectedSendSites) {
    if ($SendSites -notcontains $Expected) {
        throw "UnitreeMotorSendActuator 出现非预期所有者，缺少：$Expected"
    }
}

$DriverSource = Get-Content -LiteralPath (Join-Path $RepoRoot 'shared\application\motors\UnitreeMotorDriver.c') -Raw -Encoding UTF8
if ($DriverSource -notmatch 'LowCmdGetInhibitWriter\s*\(' -or
    $DriverSource -notmatch 'UnitreeMotorCmdSnapshotAllowed\s*\(') {
    throw 'Unitree 发送前缺少 latest LowCmd/inhibit 二次复核。'
}
if ($DriverSource -notmatch 'UnitreeMotorBrakeRequired\s*\(' -or
    $DriverSource -notmatch 'UNITREE_MOTOR_MODE_BRAKE') {
    throw 'Unitree Disable 未明确映射为物理 BRAKE。'
}

$RouteSource = Get-Content -LiteralPath (Join-Path $RepoRoot 'shared\application\comm\can\CanCommandTxRouteHelpers.inc') -Raw -Encoding UTF8
if ($RouteSource -notmatch 'ArmJ0Unitree\.control_period_ms' -or
    $RouteSource -notmatch 'UnitreeMotorTxDue\s*\(' -or
    $RouteSource -notmatch 'UnitreeMotorTxMark\s*\(') {
    throw 'Unitree 单发送路径未消费 J0 配置周期或缺少发送成功后的节流状态。'
}
if ($RouteSource -notmatch '(?s)CanTxRecheckUnitreeAuthority\s*\(actuator_id,\s*&cmd.*?UnitreeMotorTxDue\s*\(') {
    throw 'Unitree 节流前缺少 latest/inhibit 重校验，旧命令可能延迟 BRAKE。'
}

Write-Host 'PASS: Unitree 单发送者静态约束'
