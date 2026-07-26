param(
    [string]$OutputPath
)

$ErrorActionPreference = "Stop"

$RepoRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot ".."))
if ([string]::IsNullOrWhiteSpace($OutputPath)) {
    $OutputPath = Join-Path $RepoRoot "shared\generated\build_info_autogen.h"
}
else {
    if (-not [System.IO.Path]::IsPathRooted($OutputPath)) {
        $OutputPath = [System.IO.Path]::GetFullPath((Join-Path $RepoRoot $OutputPath))
    }
}

function Escape-CString {
    param([string]$Value)

    if ($null -eq $Value) {
        $Value = ""
    }

    return $Value.Replace("\", "\\").Replace('"', '\"')
}

function Invoke-GitText {
    param([string[]]$GitArgs)

    $git = Get-Command git -ErrorAction SilentlyContinue
    if ($null -eq $git) {
        return $null
    }

    $previousErrorActionPreference = $ErrorActionPreference
    try {
        # Windows PowerShell 5.1 会把原生程序的 stderr 包装成错误记录。
        # Git 的换行提示不应中断构建信息生成。
        $ErrorActionPreference = "Continue"
        $output = & $git.Source -C $RepoRoot @GitArgs 2>$null
        $gitExitCode = $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $previousErrorActionPreference
    }

    if ($gitExitCode -ne 0) {
        return $null
    }

    return (($output | Out-String).Trim())
}

$insideGit = ((Invoke-GitText @("rev-parse", "--is-inside-work-tree")) -eq "true")

$gitSha = $null
if ($insideGit) {
    $gitSha = Invoke-GitText @("rev-parse", "--short=16", "HEAD")
}
if ([string]::IsNullOrWhiteSpace($gitSha)) {
    $gitSha = "unknown"
}

$dirty = 0
$git = Get-Command git -ErrorAction SilentlyContinue
if ($insideGit -and $null -ne $git) {
    $previousErrorActionPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = "Continue"
        & $git.Source -C $RepoRoot diff --quiet --ignore-submodules -- 2>$null
        $worktreeDirty = ($LASTEXITCODE -ne 0)
        & $git.Source -C $RepoRoot diff --cached --quiet --ignore-submodules -- 2>$null
        $indexDirty = ($LASTEXITCODE -ne 0)
    }
    finally {
        $ErrorActionPreference = $previousErrorActionPreference
    }

    if ($worktreeDirty -or $indexDirty) {
        $dirty = 1
    }
}

$now = Get-Date
$buildDate = $now.ToString("yyyy-MM-dd")
$buildTime = $now.ToString("HH:mm:ss")

$dir = Split-Path -Parent $OutputPath
if (-not (Test-Path -LiteralPath $dir -PathType Container)) {
    New-Item -ItemType Directory -Path $dir | Out-Null
}

$content = @"
#ifndef ARBATOS_BUILD_INFO_AUTOGEN_H
#define ARBATOS_BUILD_INFO_AUTOGEN_H

#define ARBATOS_GIT_SHA "$(Escape-CString $gitSha)"
#define ARBATOS_BUILD_DIRTY ${dirty}u
#define ARBATOS_BUILD_DATE "$(Escape-CString $buildDate)"
#define ARBATOS_BUILD_TIME "$(Escape-CString $buildTime)"

#endif
"@

if (-not $content.EndsWith("`r`n")) {
    $content += "`r`n"
}

[System.IO.File]::WriteAllText($OutputPath, $content, [System.Text.Encoding]::ASCII)
Write-Host "generated $(Resolve-Path -LiteralPath $OutputPath)"
exit 0
