param(
    [string]$ZigPath
)

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$ZigExe = $ZigPath
if ([string]::IsNullOrWhiteSpace($ZigExe)) {
    $Zig = Get-Command zig -ErrorAction SilentlyContinue
    if ($null -eq $Zig) {
        $DefaultZigPath = Join-Path $env:LOCALAPPDATA 'Microsoft\WinGet\Links\zig.exe'
        if (Test-Path -LiteralPath $DefaultZigPath -PathType Leaf) {
            $Zig = Get-Item -LiteralPath $DefaultZigPath
        }
        else {
            throw '找不到 zig，无法运行 bxCAN W1C 回归。'
        }
    }
    $ZigExe = $Zig.Source
    if ([string]::IsNullOrWhiteSpace($ZigExe)) {
        $ZigExe = $Zig.FullName
    }
}
if (-not (Test-Path -LiteralPath $ZigExe -PathType Leaf)) {
    throw "zig 路径不存在：$ZigExe"
}

$BuildDir = Join-Path $RepoRoot 'local\build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'can-stm32-bxcan-w1c-regression.exe'
$TestSource = Join-Path $RepoRoot 'tools\tests\CanStm32BxcanW1cRegression.cpp'
$DriverSource = Join-Path $RepoRoot 'local\cache\zephyrproject\zephyr\drivers\can\can_stm32_bxcan.c'
$ExtractedSource = Join-Path $BuildDir 'CanStm32BxcanW1cActual.inc'
$SourceText = [IO.File]::ReadAllText($DriverSource)
function Get-StaticFunction([string]$Text, [string]$Name) {
    $Match = [regex]::Match(
        $Text,
        "(?ms)^static(?: inline)? [A-Za-z0-9_]+ $Name\([\s\S]*?\)\r?\n\{.*?^\}")
    if (-not $Match.Success) {
        throw "未能从实际 bxCAN 驱动提取 $Name。"
    }
    return $Match.Value
}

$BaselineText = ((git -C (Join-Path $RepoRoot 'local\cache\zephyrproject\zephyr') show 'HEAD:drivers/can/can_stm32_bxcan.c') -join [Environment]::NewLine)
$SignalFunction = Get-StaticFunction $SourceText 'can_stm32_signal_tx_complete'
$ActualFunction = Get-StaticFunction $SourceText 'can_stm32_tx_isr_handler'
$HelperFunction = Get-StaticFunction $SourceText 'can_stm32_get_tx_mailboxes'
$SendFunction = Get-StaticFunction $SourceText 'can_stm32_send'
$OriginalSignalFunction = Get-StaticFunction $BaselineText 'can_stm32_signal_tx_complete'
$OriginalFunction = Get-StaticFunction $BaselineText 'can_stm32_tx_isr_handler'
$OriginalSendFunction = Get-StaticFunction $BaselineText 'can_stm32_send'
$RequiredLines = @(
    'can->TSR = CAN_TSR_RQCP0;',
    'can->TSR = CAN_TSR_RQCP1;',
    'can->TSR = CAN_TSR_RQCP2;'
)
foreach ($Line in $RequiredLines) {
    if (-not $ActualFunction.Contains($Line)) {
        throw "实际 ISR 未包含预期的修正行：$Line"
    }
}

$FixedSignalFunction = $SignalFunction.Replace('can_stm32_signal_tx_complete', 'CanStm32SignalTxCompleteFixed')
$FixedFunction = $ActualFunction.Replace('can_stm32_tx_isr_handler', 'CanStm32TxIsrHandlerFixed').Replace('can_stm32_signal_tx_complete', 'CanStm32SignalTxCompleteFixed')
$FixedHelperFunction = $HelperFunction.Replace('can_stm32_get_tx_mailboxes', 'CanStm32GetTxMailboxesFixed')
$FixedSendFunction = $SendFunction.Replace('can_stm32_send', 'CanStm32SendFixed').Replace('can_stm32_get_tx_mailboxes', 'CanStm32GetTxMailboxesFixed')
$OriginalSignalFunction = $OriginalSignalFunction.Replace('can_stm32_signal_tx_complete', 'CanStm32SignalTxCompleteOriginal')
$OriginalFunction = $OriginalFunction.Replace('can_stm32_tx_isr_handler', 'CanStm32TxIsrHandlerOriginal').Replace('can_stm32_signal_tx_complete', 'CanStm32SignalTxCompleteOriginal')
$OriginalSendFunction = $OriginalSendFunction.Replace('can_stm32_send', 'CanStm32SendOriginal')
[IO.File]::WriteAllText(
    $ExtractedSource,
    $FixedSignalFunction + "`n`n" + $FixedFunction + "`n`n" +
    $FixedHelperFunction + "`n`n" + $FixedSendFunction + "`n`n" +
    $OriginalSignalFunction + "`n`n" + $OriginalFunction + "`n`n" +
    $OriginalSendFunction,
    [Text.UTF8Encoding]::new($false))

& $ZigExe c++ -std=c++17 -Wall -Wextra -Werror "-I$BuildDir" $TestSource -o $Output
if ($LASTEXITCODE -ne 0) {
    throw "bxCAN W1C 回归编译失败，退出码 $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "bxCAN W1C 回归失败，退出码 $LASTEXITCODE"
}
