param(
    [ValidateSet("build", "check", "probe", "sim", "legacy-check", "legacy-manifest", "legacy-gcc", "legacy-gcc-build")]
    [string]$Action = "build",

    [string]$Project = "HERO-M",

    [switch]$Pristine,

    [string]$BuildRoot,

    [string]$West,

    [string]$Ninja,

    [ValidateRange(1, 64)]
    [int]$Jobs = 2,

    [switch]$Json,

    [switch]$FailOnGccBlockers,

    [switch]$FailOnRisk
)

$ErrorActionPreference = "Stop"

$RepoRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot ".."))
$ProjectMap = [ordered]@{
    "HERO-C" = "hero-c"
    "HERO-M" = "hero-m"
    "INFANTRY-A" = "infantry-a"
    "SENTINEL-M" = "sentinel-m"
    "CARRIER-A" = "carrier-a"
    "MINIWHEELEG-M" = "miniwheeleg-m"
    "MINIWHEELEG-C" = "miniwheeleg-c"
}

function Find-Tool {
    param([string]$Name, [string[]]$PreferredPaths = @())

    foreach ($path in $PreferredPaths) {
        if ($path -and (Test-Path -LiteralPath $path -PathType Leaf -ErrorAction SilentlyContinue)) {
            return [System.IO.Path]::GetFullPath($path)
        }
    }
    $command = Get-Command $Name -ErrorAction SilentlyContinue
    if ($null -ne $command) {
        return $command.Source
    }
    return $null
}

function Resolve-Tool {
    param([string]$Value, [string]$Name, [string[]]$PreferredPaths = @())

    if ($Value) {
        if (Test-Path -LiteralPath $Value -PathType Leaf) {
            return [System.IO.Path]::GetFullPath($Value)
        }
        $command = Get-Command $Value -ErrorAction SilentlyContinue
        if ($null -ne $command) {
            return $command.Source
        }
        throw "Cannot find ${Name}: $Value. Provide a valid path or executable command."
    }
    $found = Find-Tool -Name $Name -PreferredPaths $PreferredPaths
    if ($null -eq $found) {
        throw "Cannot find $Name. Run tools\\build.ps1 -Action probe, or provide its path."
    }
    return $found
}

function Resolve-ZephyrProject {
    if ($Project -ieq "all") {
        return "all"
    }
    foreach ($key in $ProjectMap.Keys) {
        if ($Project -ieq $key) {
            return $ProjectMap[$key]
        }
    }
    throw "Unknown project: $Project. Use all or one of: $($ProjectMap.Keys -join ', ')."
}

function Invoke-PythonTool {
    param(
        [string]$ToolPath,
        [string[]]$Arguments
    )

    $localPython = Join-Path $RepoRoot "local\cache\zephyrproject\.venv\Scripts\python.exe"
    $python = Resolve-Tool -Name "python" -PreferredPaths @($localPython)
    if (-not (Test-Path -LiteralPath $ToolPath -PathType Leaf)) {
        throw "Cannot find tool: $ToolPath"
    }
    & $python $ToolPath @Arguments
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

function Show-ResolvedTool {
    param([string]$Name, [string]$Path, [string[]]$VersionArgs = @())

    if (-not $Path) {
        Write-Host ("{0}: missing" -f $Name)
        return
    }
    Write-Host ("{0}: {1}" -f $Name, $Path)
    if ($VersionArgs.Count -gt 0) {
        try {
            $version = & $Path @VersionArgs 2>&1 | Select-Object -First 1
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
    "build" {
        $target = Resolve-ZephyrProject
        if ([string]::IsNullOrWhiteSpace($BuildRoot)) {
            $BuildRoot = Join-Path $RepoRoot "out\zephyr"
        }
        $localVenv = Join-Path $RepoRoot "local\cache\zephyrproject\.venv\Scripts"
        $westPath = Resolve-Tool -Value $West -Name "West" -PreferredPaths @(Join-Path $localVenv "west.exe")
        $ninjaPath = Resolve-Tool -Value $Ninja -Name "Ninja" -PreferredPaths @(Join-Path $localVenv "ninja.exe")
        if ($Pristine) {
            & (Join-Path $RepoRoot "zephyr\scripts\build-matrix.ps1") -Target $target -BuildRoot $BuildRoot -West $westPath -Ninja $ninjaPath -Jobs $Jobs -Pristine
        }
        else {
            & (Join-Path $RepoRoot "zephyr\scripts\build-matrix.ps1") -Target $target -BuildRoot $BuildRoot -West $westPath -Ninja $ninjaPath -Jobs $Jobs
        }
        exit $LASTEXITCODE
    }

    "check" {
        $arguments = @("--project", $Project)
        if ($Json) {
            $arguments += "--json"
        }
        Invoke-PythonTool -ToolPath (Join-Path $RepoRoot "tools\CheckZephyr.py") -Arguments $arguments
    }

    "legacy-check" {
        & powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $RepoRoot "tools\CheckAll.ps1")
        exit $LASTEXITCODE
    }

    "legacy-manifest" {
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

    "legacy-gcc" {
        Update-BuildInfo
        Invoke-GccGenerator
        exit 0
    }

    "legacy-gcc-build" {
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
            Write-Host ("[legacy-gcc-build] {0}: configure" -f $projectName)
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

            Write-Host ("[legacy-gcc-build] {0}: build" -f $projectName)
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

        $localPython = Join-Path $RepoRoot "local\cache\zephyrproject\.venv\Scripts\python.exe"
        $python = Resolve-Tool -Name "python" -PreferredPaths @($localPython)

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

            & $python $simTool @($arguments.ToArray())
            if ($LASTEXITCODE -ne 0) {
                $exitCode = $LASTEXITCODE
            }
        }

        exit $exitCode
    }

    "probe" {
        $localVenv = Join-Path $RepoRoot "local\cache\zephyrproject\.venv\Scripts"
        $sdkRoot = Join-Path $RepoRoot "local\cache\zephyr-sdk"
        Write-Host "ARBATOS Zephyr build environment"
        Write-Host ("repo: {0}" -f $RepoRoot)
        Show-ResolvedTool "west" (Find-Tool -Name "west" -PreferredPaths @(Join-Path $localVenv "west.exe")) @("--version")
        Show-ResolvedTool "python" (Find-Tool -Name "python" -PreferredPaths @(Join-Path $localVenv "python.exe")) @("--version")
        Show-ResolvedTool "cmake" (Find-Tool -Name "cmake") @("--version")
        Show-ResolvedTool "ninja" (Find-Tool -Name "ninja" -PreferredPaths @(Join-Path $localVenv "ninja.exe")) @("--version")
        Show-ResolvedTool "openocd" (Find-Tool -Name "openocd" -PreferredPaths @(Join-Path $sdkRoot "hosttools\openocd\bin\openocd.exe")) @("--version")
        $zephyrBase = if ($env:ZEPHYR_BASE) { $env:ZEPHYR_BASE } else { Join-Path $RepoRoot "local\cache\zephyrproject\zephyr" }
        $sdk = if ($env:ZEPHYR_SDK_INSTALL_DIR) { $env:ZEPHYR_SDK_INSTALL_DIR } else { $sdkRoot }
        if (Test-Path -LiteralPath $zephyrBase) { Write-Host "ZEPHYR_BASE: $zephyrBase" } else { Write-Host "ZEPHYR_BASE: missing ($zephyrBase)" }
        if (Test-Path -LiteralPath $sdk) { Write-Host "ZEPHYR_SDK_INSTALL_DIR: $sdk" } else { Write-Host "ZEPHYR_SDK_INSTALL_DIR: missing ($sdk)" }
    }
}
