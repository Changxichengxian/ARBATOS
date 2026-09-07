param(
    [string]$OutputPath
)

$ErrorActionPreference = "Stop"
$RepoRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot "..\.."))
if ([string]::IsNullOrWhiteSpace($OutputPath)) {
    $OutputPath = Join-Path $RepoRoot "shared\generated\build_info_autogen.h"
}
elseif (-not [System.IO.Path]::IsPathRooted($OutputPath)) {
    $OutputPath = [System.IO.Path]::GetFullPath((Join-Path $RepoRoot $OutputPath))
}

$Python = Get-Command python -ErrorAction SilentlyContinue
if ($null -eq $Python) {
    $Python = Get-Command py -ErrorAction SilentlyContinue
    if ($null -eq $Python) {
        $PythonExe = Get-ChildItem -Path (Join-Path $env:LOCALAPPDATA "Programs\Python") -Filter python.exe -File -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($null -eq $PythonExe) {
            throw "找不到 Python，无法生成构建版本信息。"
        }
        & $PythonExe.FullName (Join-Path $PSScriptRoot "GenBuildInfo.py") --repo $RepoRoot --output $OutputPath
        exit $LASTEXITCODE
    }
    & $Python.Source -3 (Join-Path $PSScriptRoot "GenBuildInfo.py") --repo $RepoRoot --output $OutputPath
}
else {
    & $Python.Source (Join-Path $PSScriptRoot "GenBuildInfo.py") --repo $RepoRoot --output $OutputPath
}
exit $LASTEXITCODE
