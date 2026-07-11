param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw 'zig is required for the WheelLeg output plan host regression.'
}

$BuildDir = Join-Path $RepoRoot 'build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'wheelleg-output-plan-regression.exe'
$TestSource = Join-Path $RepoRoot 'tools\tests\WheelLegOutputPlanRegression.c'
$IncludeDirs = @(
    (Join-Path $RepoRoot 'shared\components\support'),
    (Join-Path $RepoRoot 'shared\application\robot'),
    (Join-Path $RepoRoot 'shared\application\wheelleg')
)

$Args = @('cc', '-std=c99', '-Wall', '-Wextra', '-Werror')
foreach ($Dir in $IncludeDirs) {
    $Args += "-I$Dir"
}
$Args += @($TestSource, '-o', $Output)

& $Zig.Source @Args
if ($LASTEXITCODE -ne 0) {
    throw "WheelLeg output plan host regression compile failed: $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "WheelLeg output plan host regression failed: $LASTEXITCODE"
}
