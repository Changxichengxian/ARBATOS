param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw 'zig is required for service input policy regression.'
}

$TaskFiles = @(
    'shared\application\arm\ArmTask.c',
    'shared\application\services\servo\ServoControlTask.c',
    'shared\application\services\calibration\CalibrateTask.c'
)
$Forbidden = 'ManualInputGetCurrentCopy|\binput_axis\s*\(|\binput_switch\s*\(|DBUS_TOE'
foreach ($TaskFile in $TaskFiles) {
    $Content = Get-Content -LiteralPath (Join-Path $RepoRoot $TaskFile) -Raw -Encoding UTF8
    if ($Content -match $Forbidden) {
        throw "$TaskFile still uses a legacy input reader."
    }
    if ([regex]::Matches($Content, 'ManualInputSnapshotRead\s*\(').Count -ne 1) {
        throw "$TaskFile must keep exactly one ManualInputSnapshotRead call."
    }
}

$CalibrateSource = Get-Content -LiteralPath (
    Join-Path $RepoRoot 'shared\application\services\calibration\CalibrateTask.c'
) -Raw -Encoding UTF8
foreach ($Field in @(
        'axis[INPUT_AXIS_CALIB_0]',
        'axis[INPUT_AXIS_CALIB_1]',
        'axis[INPUT_AXIS_CALIB_2]',
        'axis[INPUT_AXIS_CALIB_3]',
        'sw[INPUT_SW_CALIB_L]',
        'sw[INPUT_SW_CALIB_R]')) {
    if (-not $CalibrateSource.Contains($Field)) {
        throw "CalibrateTask.c does not read $Field from its aggregate snapshot."
    }
}
if ($CalibrateSource -notmatch 'manualInput\.online\s*==\s*0u') {
    throw 'CalibrateTask.c is missing its same-frame offline guard.'
}

$BuildDir = Join-Path $RepoRoot 'build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'service-input-policy-regression.exe'
$Source = Join-Path $RepoRoot 'tools\tests\ServiceInputPolicyRegression.c'
$Args = @(
    'cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
    ('-I' + (Join-Path $RepoRoot 'shared\application\arm')),
    ('-I' + (Join-Path $RepoRoot 'shared\application\chassis')),
    ('-I' + (Join-Path $RepoRoot 'shared\application\gimbal')),
    ('-I' + (Join-Path $RepoRoot 'shared\application\services\servo')),
    $Source, '-o', $Output
)

& $Zig.Source @Args
if ($LASTEXITCODE -ne 0) {
    throw "service input policy regression failed to compile: $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "service input policy regression failed: $LASTEXITCODE"
}
