param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw 'zig is required for reset evidence port regression.'
}

$BuildDir = Join-Path $RepoRoot 'local\build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'reset-evidence-port-regression.exe'
$TestSource = Join-Path $RepoRoot 'tools\tests\ResetEvidencePortRegression.c'
$StubDir = Join-Path $RepoRoot 'tools\tests\reset-evidence-port-stubs'
$HalDir = Join-Path $RepoRoot 'shared\hal'

& $Zig.Source cc -std=c99 -Wall -Wextra -Werror "-I$StubDir" "-I$HalDir" $TestSource -o $Output
if ($LASTEXITCODE -ne 0) {
    throw "Reset evidence port regression compile failed: $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "Reset evidence port regression failed: $LASTEXITCODE"
}
