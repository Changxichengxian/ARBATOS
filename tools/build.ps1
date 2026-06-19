param(
    [ValidateSet("check", "manifest", "probe", "sim", "gcc", "gcc-build")]
    [string]$Action = "check",

    [string]$Project = "all",

    [switch]$Json,

    [switch]$FailOnGccBlockers,

    [switch]$FailOnRisk
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

function Require-Tool {
    param(
        [string]$Name
    )

    $command = Get-Command $Name -ErrorAction SilentlyContinue
    if ($null -eq $command) {
        Write-Error ("{0} is not available. Run tools\build.ps1 -Action probe to inspect the toolchain." -f $Name)
    }

    return $command.Source
}

function Get-SelectedGccProjects {
    if ($Project -eq "all") {
        return @(Get-ChildItem -Path (Join-Path $RepoRoot "build\gcc") -Directory |
            Sort-Object Name |
            ForEach-Object { $_.Name })
    }

    return @($Project)
}

function Invoke-GccGenerator {
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

    $python = Require-Tool "python"
    & $python (Join-Path $RepoRoot "tools\build\GccProject.py") @($arguments.ToArray())
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}

function Update-BuildInfo {
    & powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $RepoRoot "tools\GenBuildInfo.ps1")
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}

switch ($Action) {
    "check" {
        & powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $RepoRoot "tools\CheckAll.ps1")
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

        Invoke-PythonTool -ToolPath (Join-Path $RepoRoot "tools\build\ProjectManifest.py") -Arguments $arguments.ToArray()
    }

    "gcc" {
        Update-BuildInfo
        Invoke-GccGenerator
        exit 0
    }

    "gcc-build" {
        Update-BuildInfo
        Invoke-GccGenerator
        $cmake = Require-Tool "cmake"
        $ninja = Require-Tool "ninja"
        Require-Tool "arm-none-eabi-gcc" | Out-Null

        $exitCode = 0
        foreach ($projectName in Get-SelectedGccProjects) {
            $sourceDir = Join-Path $RepoRoot ("build\gcc\{0}" -f $projectName)
            $buildDir = Join-Path $sourceDir "build"
            $toolchainFile = Join-Path $sourceDir "arm-none-eabi-gcc.cmake"
            $cacheFile = Join-Path $buildDir "CMakeCache.txt"

            Write-Host ""
            Write-Host ("[gcc-build] {0}: configure" -f $projectName)
            if (Test-Path $cacheFile) {
                & $cmake -S $sourceDir -B $buildDir -G Ninja
            }
            else {
                & $cmake -S $sourceDir -B $buildDir -G Ninja "-DCMAKE_TOOLCHAIN_FILE=$toolchainFile" "-DCMAKE_MAKE_PROGRAM=$ninja"
            }
            if ($LASTEXITCODE -ne 0) {
                $exitCode = $LASTEXITCODE
                continue
            }

            Write-Host ("[gcc-build] {0}: build" -f $projectName)
            & $cmake --build $buildDir
            if ($LASTEXITCODE -ne 0) {
                $exitCode = $LASTEXITCODE
            }
        }

        exit $exitCode
    }

    "sim" {
        if ($Json -and $Project -eq "all") {
            Write-Error "-Json is only supported with -Project <name> for sim output."
        }

        $python = Get-Command python -ErrorAction SilentlyContinue
        if ($null -eq $python) {
            Write-Error "python is not available. Install Python or run tools\sim\RobotSim.py directly."
        }

        $simTool = Join-Path $RepoRoot "tools\sim\RobotSim.py"
        $projects = @()
        if ($Project -eq "all") {
            $projects = @(Get-ChildItem -Path (Join-Path $RepoRoot "Robotconfig") -Directory |
                Sort-Object Name |
                ForEach-Object { $_.Name })
        }
        else {
            $projects = @($Project)
        }

        $exitCode = 0
        foreach ($projectName in $projects) {
            $arguments = New-Object System.Collections.Generic.List[string]
            $arguments.Add("--project")
            $arguments.Add($projectName)
            if ($Json) {
                $arguments.Add("--json")
            }
            if ($FailOnRisk) {
                $arguments.Add("--fail-on-risk")
            }

            if ($projects.Count -gt 1) {
                Write-Host ""
                Write-Host ("[sim] {0}" -f $projectName)
            }

            & $python.Source $simTool @($arguments.ToArray())
            if ($LASTEXITCODE -ne 0) {
                $exitCode = $LASTEXITCODE
            }
        }

        exit $exitCode
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
