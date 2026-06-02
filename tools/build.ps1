param(
    [ValidateSet("check", "manifest", "probe")]
    [string]$Action = "check",

    [string]$Project = "all",

    [switch]$Json,

    [switch]$FailOnGccBlockers
)

$ErrorActionPreference = "Stop"

$RepoRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot ".."))

function Invoke-PythonTool {
    param(
        [string]$ToolPath,
        [string[]]$Arguments
    )

    $python = Get-Command python -ErrorAction SilentlyContinue
    if ($null -eq $python) {
        Write-Error "python is not available. Install Python or run the Keil project directly."
    }

    & $python.Source $ToolPath @Arguments
    exit $LASTEXITCODE
}

function Show-Tool {
    param(
        [string]$Name,
        [string[]]$VersionArgs
    )

    $command = Get-Command $Name -ErrorAction SilentlyContinue
    if ($null -eq $command) {
        Write-Host ("{0}: missing" -f $Name)
        return
    }

    Write-Host ("{0}: {1}" -f $Name, $command.Source)
    if ($VersionArgs.Count -gt 0) {
        try {
            $version = & $command.Source @VersionArgs 2>&1 | Select-Object -First 1
            if ($LASTEXITCODE -eq 0 -and -not [string]::IsNullOrWhiteSpace($version)) {
                Write-Host ("  {0}" -f $version)
            }
        }
        catch {
            Write-Host ("  version check failed: {0}" -f $_.Exception.Message)
        }
    }
}

switch ($Action) {
    "check" {
        & powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $RepoRoot "tools\check_all.ps1")
        exit $LASTEXITCODE
    }

    "manifest" {
        $arguments = New-Object System.Collections.Generic.List[string]
        if ($Project -eq "all") {
            $arguments.Add("--all")
        }
        else {
            $arguments.Add("--project")
            $arguments.Add($Project)
        }

        if ($Json) {
            $arguments.Add("--json")
        }
        if ($FailOnGccBlockers) {
            $arguments.Add("--fail-on-gcc-blockers")
        }

        Invoke-PythonTool -ToolPath (Join-Path $RepoRoot "tools\build\project_manifest.py") -Arguments $arguments.ToArray()
    }

    "probe" {
        Write-Host "ARBATOS build tool probe"
        Write-Host ("repo: {0}" -f $RepoRoot)
        Show-Tool "python" @("--version")
        Show-Tool "arm-none-eabi-gcc" @("--version")
        Show-Tool "cmake" @("--version")
        Show-Tool "ninja" @("--version")
        Show-Tool "UV4" @()
        Show-Tool "UV4.exe" @()
    }
}
