# SPDX-License-Identifier: Apache-2.0
param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$RepoWsl = (& wsl.exe wslpath -a ($RepoRoot -replace '\\', '/')).Trim()
if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($RepoWsl)) {
    throw 'Unable to resolve the repository path in WSL.'
}

$BuildDir = "$RepoWsl/local/build/host-tests"
$Output = "$BuildDir/hi14-parser-regression"
$Source = "$RepoWsl/tools/tests/Hi14ParserRegression.c"
$Parser = "$RepoWsl/shared/zephyr/port/sensors/Hi14Parser.c"
$Include = "$RepoWsl/shared/zephyr/port/sensors"

& wsl.exe sh -lc "mkdir -p '$BuildDir' && cc -std=c99 -Wall -Wextra -Werror -I'$Include' '$Source' '$Parser' -lm -o '$Output' && '$Output'"
if ($LASTEXITCODE -ne 0) {
    throw "HI14 parser regression failed with exit code $LASTEXITCODE"
}
