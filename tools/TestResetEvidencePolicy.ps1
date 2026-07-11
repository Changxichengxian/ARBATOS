param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw 'zig is required for reset evidence policy regression.'
}

$BuildDir = Join-Path $RepoRoot 'build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'reset-evidence-policy-regression.exe'
$TestSource = Join-Path $RepoRoot 'tools\tests\ResetEvidencePolicyRegression.c'
$IncludeDir = Join-Path $RepoRoot 'shared\hal'

& $Zig.Source cc -std=c99 -Wall -Wextra -Werror "-I$IncludeDir" $TestSource -o $Output
if ($LASTEXITCODE -ne 0) {
    throw "Reset evidence policy regression compile failed: $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "Reset evidence policy regression failed: $LASTEXITCODE"
}
