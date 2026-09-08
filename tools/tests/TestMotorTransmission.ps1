param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$BuildDir = Join-Path $RepoRoot 'local\build\host-tests'
$Source = Join-Path $RepoRoot 'tools\tests\MotorTransmissionRegression.c'
$IncludeDir = Join-Path $RepoRoot 'shared\components\support'
$MotorIncludeDir = Join-Path $RepoRoot 'shared\application\motors'
$Output = Join-Path $BuildDir 'motor-transmission-regression'

New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null

$Compiler = Get-Command cc -ErrorAction SilentlyContinue
if ($null -ne $Compiler) {
    & $Compiler.Source -std=c99 -Wall -Wextra -Werror `
        "-I$IncludeDir" "-I$MotorIncludeDir" $Source -o $Output
    if ($LASTEXITCODE -ne 0) {
        throw "外置减速器回归编译失败，退出码 $LASTEXITCODE"
    }
    & $Output
}
else {
    $Wsl = Get-Command wsl.exe -ErrorAction SilentlyContinue
    if ($null -eq $Wsl) {
        throw '找不到本机 cc 或 WSL cc，无法运行外置减速器回归。'
    }

    & $Wsl.Source --cd $RepoRoot cc -std=c99 -Wall -Wextra -Werror `
        -Ishared/components/support -Ishared/application/motors `
        tools/tests/MotorTransmissionRegression.c `
        -o local/build/host-tests/motor-transmission-regression
    if ($LASTEXITCODE -ne 0) {
        throw "外置减速器回归编译失败，退出码 $LASTEXITCODE"
    }
    & $Wsl.Source --cd $RepoRoot ./local/build/host-tests/motor-transmission-regression
}

if ($LASTEXITCODE -ne 0) {
    throw "外置减速器回归失败，退出码 $LASTEXITCODE"
}
