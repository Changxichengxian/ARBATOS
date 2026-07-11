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
if ($DriverSource -notmatch 'UnitreeMotorBrakeRequired\s*\(' -or
    $DriverSource -notmatch 'UNITREE_MOTOR_MODE_BRAKE') {
    throw 'Unitree Disable 未明确映射为物理 BRAKE。'
}

$N6014bHeaderSource = Get-Content -LiteralPath (Join-Path $RepoRoot 'shared\application\motors\N6014bMotorDriver.h') -Raw -Encoding UTF8
$UnitreeHeaderSource = Get-Content -LiteralPath (Join-Path $RepoRoot 'shared\application\motors\UnitreeMotorDriver.h') -Raw -Encoding UTF8
$N6014bDriverSource = Get-Content -LiteralPath (Join-Path $RepoRoot 'shared\application\motors\N6014bMotorDriver.c') -Raw -Encoding UTF8

if ($UnitreeHeaderSource -notmatch '(?s)UnitreeMotorSendActuator\s*\(.*?const ControlOutputStamp\s*\*owner\s*\)') {
    throw 'Unitree 公共发送接口未携带控制输出 owner。'
}
if ($N6014bHeaderSource -notmatch '(?s)N6014bMotorSendActuator\s*\(.*?const ControlOutputStamp\s*\*owner\s*\)') {
    throw 'N6014b 公共发送接口未携带控制输出 owner。'
}
if ($N6014bDriverSource -notmatch '(?s)CanTxProcessExtraItem\s*\(.*?const ControlOutputStamp\s*\*owner\s*\).*?UnitreeMotorSendActuator\s*\(.*?cmd,\s*owner\s*\).*?N6014bMotorSendActuator\s*\(.*?cmd,\s*owner\s*\)') {
    throw 'CanTx 扩展分发没有把同一 owner 贯穿 Unitree/N6014b。'
}
if ($DriverSource -notmatch '(?s)UnitreeMotorSendActuator\s*\(.*?const ControlOutputStamp\s*\*owner\s*\).*?UnitreeMotorSendCmd\s*\(.*?can_tx_cmd,\s*owner\s*,') {
    throw 'UnitreeMotorSendActuator 没有把 owner 传给发送链。'
}
if ($DriverSource -notmatch '(?s)UnitreeMotorSendCmd\s*\(.*?const ControlOutputStamp\s*\*owner\s*,.*?\)\s*\{.*?UnitreeMotorSendFrame\s*\(.*?cached_cmd,\s*owner\s*,') {
    throw 'UnitreeMotorSendCmd 没有把 owner 传到物理发送边界。'
}
if ($N6014bDriverSource -notmatch '(?s)N6014bMotorSendActuator\s*\(.*?const ControlOutputStamp\s*\*owner\s*\).*?N6014bTx\s*\(.*?cmd,\s*owner\s*,') {
    throw 'N6014bMotorSendActuator 没有把 owner 传到物理发送边界。'
}

function Assert-FinalTxBoundary {
    param(
        [string]$Name,
        [string]$Source,
        [string]$FunctionPattern,
        [string]$AuthorityPattern,
        [string]$StartPattern,
        [string]$WaitPattern
    )

    $Match = [regex]::Match($Source,
        "(?ms)^static int $FunctionPattern\([^;]+?\)\s*\{(?<body>.*?)^\}")
    if (-not $Match.Success) {
        throw "$Name 最终发送函数提取失败。"
    }
    $Body = $Match.Groups['body'].Value
    $Enter = $Body.IndexOf('taskENTER_CRITICAL();', [StringComparison]::Ordinal)
    $Authority = [regex]::Match($Body, $AuthorityPattern)
    $Start = [regex]::Match($Body, $StartPattern)
    $Exit = $Body.IndexOf('taskEXIT_CRITICAL();', [StringComparison]::Ordinal)
    $Wait = [regex]::Match($Body, $WaitPattern)
    if ($Enter -lt 0 -or -not $Authority.Success -or -not $Start.Success -or
        $Exit -lt 0 -or -not $Wait.Success -or
        $Enter -ge $Authority.Index -or $Authority.Index -ge $Start.Index -or
        $Start.Index -ge $Exit -or $Exit -ge $Wait.Index) {
        throw "$Name 最终 owner 校验、非阻塞 TxStart、退出临界区和 Wait 的顺序被破坏。"
    }
    $Critical = $Body.Substring($Enter, $Exit - $Enter)
    if ([regex]::IsMatch($Critical, $WaitPattern)) {
        throw "$Name 串口 Wait 进入 task 临界区，可能阻塞实时任务。"
    }
}

Assert-FinalTxBoundary `
    -Name 'Unitree' `
    -Source $DriverSource `
    -FunctionPattern 'UnitreeMotorSendFrame' `
    -AuthorityPattern 'UnitreeMotorActiveFrameAllowed\s*\(actuator_id,\s*cached_cmd,\s*owner\s*\)' `
    -StartPattern 'UnitreeMotorStartFrame\s*\(' `
    -WaitPattern 'UnitreeMotorWaitFrame\s*\('

Assert-FinalTxBoundary `
    -Name 'N6014b' `
    -Source $N6014bDriverSource `
    -FunctionPattern 'N6014bTx' `
    -AuthorityPattern 'N6014bActiveFrameAllowed\s*\(actuator_id,\s*cached_cmd,\s*owner\s*\)' `
    -StartPattern 'N6014bTxStart\s*\(' `
    -WaitPattern 'N6014bTxWait\s*\('

foreach ($Boundary in @(
    @{ Name = 'Unitree'; Source = $DriverSource; Pattern = '(?s)static uint8_t UnitreeMotorActiveFrameAllowed\s*\(.*?^\}' },
    @{ Name = 'N6014b'; Source = $N6014bDriverSource; Pattern = '(?s)static uint8_t N6014bActiveFrameAllowed\s*\(.*?^\}' }
)) {
    $Match = [regex]::Match($Boundary.Source, $Boundary.Pattern, [Text.RegularExpressions.RegexOptions]::Multiline)
    if (-not $Match.Success -or
        $Match.Value -notmatch 'RobotSafetyOutputLocked\s*\(\)\s*==\s*0u' -or
        $Match.Value -notmatch 'LowCmdOutputSnapshotAuthorized\s*\(actuator_id,\s*cached_cmd,\s*owner\s*\)') {
        throw "$($Boundary.Name) 最终活动帧校验未同时检查全局输出锁、命令快照和 owner。"
    }
    if ($Match.Value -match 'LowCmdGetMotor\s*\(' -or
        $Match.Value -match 'LowCmdGetInhibitWriter\s*\(' -or
        $Match.Value -match 'LowCmdSnapshotAuthorized\s*\(') {
        throw "$($Boundary.Name) 最终边界仍绕过带 owner 的统一授权接口。"
    }
}

if ($DriverSource -notmatch 'UnitreeMotorOutputAuthorityRequired\s*\(applied_mode\s*\)' -or
    $N6014bDriverSource -notmatch '\(uint8_t\)\(mode\s*!=\s*N6014B_MODE_LOCK\)') {
    throw 'Disable/BRAKE 或 LOCK 的无 owner 安全发送语义不明确。'
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
