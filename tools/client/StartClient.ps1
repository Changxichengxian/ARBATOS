[CmdletBinding()]
param([switch]$BuildOnly)

$ErrorActionPreference = 'Stop'
$RepoRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$ClientRoot = Join-Path $RepoRoot 'client'
$Python = if ($env:ARBATOS_PYTHON) { $env:ARBATOS_PYTHON } else { Join-Path $RepoRoot 'local\cache\zephyrproject\.venv\Scripts\python.exe' }
if (-not (Test-Path -LiteralPath $Python -PathType Leaf)) {
    $PythonCommand = Get-Command python -ErrorAction SilentlyContinue
    if ($null -eq $PythonCommand) { throw '需要 Python 3.12 或更高版本，或现有项目虚拟环境。' }
    $Python = $PythonCommand.Source
}
$Npm = Get-Command npm.cmd -ErrorAction Stop
& $Python -c 'import sys; sys.exit(0 if sys.version_info >= (3, 12) else 1)'
if ($LASTEXITCODE -ne 0) { throw '客户端需要 Python 3.12 或更高版本。' }
& $Python -c 'import serial, tomlkit, yaml'
if ($LASTEXITCODE -ne 0) {
    & $Python -m pip install -r (Join-Path $ClientRoot 'backend\requirements.txt')
    if ($LASTEXITCODE -ne 0) { throw '安装客户端 Python 依赖失败。' }
}
Push-Location -LiteralPath $ClientRoot
try {
    $LockHash = (Get-FileHash -LiteralPath 'package-lock.json' -Algorithm SHA256).Hash
    $StampPath = Join-Path $RepoRoot 'local\cache\client\dependencies.sha256'
    $InstalledHash = if (Test-Path -LiteralPath $StampPath) { (Get-Content -Raw -Encoding UTF8 -LiteralPath $StampPath).Trim() } else { '' }
    if ($InstalledHash -ne $LockHash -or -not (Test-Path -LiteralPath 'node_modules\electron\dist\electron.exe')) {
        & $Npm.Source ci --no-audit --no-fund
        if ($LASTEXITCODE -ne 0) { throw '安装客户端依赖失败。' }
        if ($env:HTTP_PROXY -or $env:HTTPS_PROXY) {
            $env:ELECTRON_GET_USE_PROXY = '1'
            $env:NODE_USE_ENV_PROXY = '1'
        }
        & $Npm.Source run setup:desktop
        if ($LASTEXITCODE -ne 0) { throw '下载桌面运行程序失败，请检查网络后重试。' }
        New-Item -ItemType Directory -Force -Path (Split-Path -Parent $StampPath) | Out-Null
        Set-Content -Encoding UTF8 -LiteralPath $StampPath -Value $LockHash
    }
    & $Npm.Source run build
    if ($LASTEXITCODE -ne 0) { throw '客户端编译失败。' }
    if (-not $BuildOnly) {
        $env:ARBATOS_PYTHON = $Python
        Start-Process -FilePath (Join-Path $ClientRoot 'node_modules\electron\dist\electron.exe') -ArgumentList '.' -WorkingDirectory $ClientRoot -WindowStyle Hidden
    }
}
finally { Pop-Location }
