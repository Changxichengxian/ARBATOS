param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
if ($null -eq $Zig) {
    throw 'zig is required for the ELRS input regression.'
}

$BuildDir = Join-Path $RepoRoot 'build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'elrs-input-regression.exe'
$Source = Join-Path $RepoRoot 'tools\tests\ElrsInputRegression.c'
$Args = @(
    'cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
    ('-I' + (Join-Path $RepoRoot 'tools\tests\manual-input-stubs')),
    ('-I' + (Join-Path $RepoRoot 'shared\application\input')),
    $Source,
    '-o', $Output
)

& $Zig.Source @Args
if ($LASTEXITCODE -ne 0) {
    throw "ELRS input regression build failed with exit code $LASTEXITCODE"
}

& $Output
if ($LASTEXITCODE -ne 0) {
    throw "ELRS input regression failed with exit code $LASTEXITCODE"
}
