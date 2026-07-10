param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw 'zig is required for the motor-axis fault-policy regression.'
}

$BuildDir = Join-Path $RepoRoot 'build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'motor-axis-fault-policy-regression.exe'
$Sources = @(
    (Join-Path $RepoRoot 'tools\tests\MotorAxisFaultPolicyRegression.c'),
    (Join-Path $RepoRoot 'shared\components\controller\ChassisPowerLimiter.c'),
    (Join-Path $RepoRoot 'shared\application\motors\MotorModelDb.c')
)
$MotorInclude = Join-Path $RepoRoot 'shared\application\motors'
$GimbalInclude = Join-Path $RepoRoot 'shared\application\gimbal'
$RobotInclude = Join-Path $RepoRoot 'shared\application\robot'
$SupportInclude = Join-Path $RepoRoot 'shared\components\support'
$ControllerInclude = Join-Path $RepoRoot 'shared\components\controller'
$ConfigInclude = Join-Path $RepoRoot 'Robotconfig\SENTINEL-M'

& $Zig.Source cc -std=c99 -Wall -Wextra -Werror -pedantic `
    "-I$MotorInclude" "-I$GimbalInclude" "-I$RobotInclude" "-I$SupportInclude" `
    "-I$ControllerInclude" "-I$ConfigInclude" `
    $Sources -o $Output
if ($LASTEXITCODE -ne 0) {
    throw "motor-axis fault-policy regression compile failed with exit code $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "motor-axis fault-policy regression failed with exit code $LASTEXITCODE"
}
