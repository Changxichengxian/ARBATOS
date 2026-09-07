param(
    [ValidateSet("build", "check", "probe", "sim", "flash", "debug")]
    [string]$Action = "build",

    [string]$Project = "HERO-M",

    [switch]$Pristine,

    [string]$BuildRoot,

    [string]$West,

    [string]$Ninja,

    [ValidateRange(1, 64)]
    [int]$Jobs = 2,

    [switch]$Json,

    [switch]$FailOnRisk
)

$ErrorActionPreference = "Stop"

$RepoRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot ".."))
$ProjectMap = [ordered]@{
    "HERO-M" = "hero-m"
    "SENTINEL-M" = "sentinel-m"
    "MINIWHEELEG-M" = "miniwheeleg-m"
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

function Get-ExistingZephyrBuild {
    $target = Resolve-ZephyrProject
    if ($target -eq "all") {
        throw "flash/debug require one formal project. Project all is not allowed."
    }
    if ([string]::IsNullOrWhiteSpace($BuildRoot)) {
        $BuildRoot = Join-Path $RepoRoot "local\build"
    }
    $buildDir = [System.IO.Path]::GetFullPath((Join-Path $BuildRoot $target))
    $requiredFiles = @(
        (Join-Path $buildDir "CMakeCache.txt"),
        (Join-Path $buildDir "zephyr\runners.yaml"),
        (Join-Path $buildDir "zephyr\.config"),
        (Join-Path $buildDir "zephyr\zephyr.elf"),
        (Join-Path $buildDir "zephyr\zephyr.hex"),
        (Join-Path $buildDir "zephyr\zephyr.bin")
    )
    $missing = @($requiredFiles | Where-Object { -not (Test-Path -LiteralPath $_ -PathType Leaf) })
    if ($missing.Count -gt 0) {
        throw "No complete Zephyr build exists for $Project at $buildDir. Build it first with -Action build. Missing: $($missing -join ', ')"
    }
    $configPath = Join-Path $buildDir "zephyr\.config"
    $configLines = Get-Content -LiteralPath $configPath
    $targetSymbol = "CONFIG_ARBATOS_TARGET_$($target.ToUpperInvariant().Replace('-', '_'))"
    $enabledTargets = @($configLines | Where-Object { $_ -match '^CONFIG_ARBATOS_TARGET_[A-Z0-9_]+=y$' } |
        ForEach-Object { $_.Split('=')[0] })
    if ($enabledTargets.Count -ne 1 -or $enabledTargets[0] -ne $targetSymbol) {
        throw "Build configuration $configPath does not select exactly $targetSymbol; refusing flash/debug."
    }
    foreach ($modeSymbol in @('CONFIG_ARBATOS_MUSIC_ONLY', 'CONFIG_ARBATOS_PREFLIGHT_ONLY', 'CONFIG_ARBATOS_RECEIVE_ONLY')) {
        if ($configLines -contains "$modeSymbol=y") {
            throw "Build configuration $configPath enables dedicated mode $modeSymbol; refusing flash/debug."
        }
    }
    return $buildDir
}

function Get-OpenOcdTools {
    $localVenv = Join-Path $RepoRoot "local\cache\zephyrproject\.venv\Scripts"
    $localZephyrBase = Join-Path $RepoRoot "local\cache\zephyrproject\zephyr"
    if (-not $env:ZEPHYR_BASE -and (Test-Path -LiteralPath $localZephyrBase -PathType Container)) {
        $env:ZEPHYR_BASE = $localZephyrBase
    }
    if (-not $env:ZEPHYR_BASE) {
        throw "ZEPHYR_BASE is not configured and local Zephyr source is missing: $localZephyrBase"
    }
    $westPath = Resolve-Tool -Value $West -Name "West" -PreferredPaths @(Join-Path $localVenv "west.exe")
    $sdkRoot = if ($env:ZEPHYR_SDK_INSTALL_DIR) { $env:ZEPHYR_SDK_INSTALL_DIR } else { Join-Path $RepoRoot "local\cache\zephyr-sdk" }
    $openOcd = Join-Path $sdkRoot "hosttools\openocd\bin\openocd.exe"
    $gdb = Join-Path $sdkRoot "gnu\arm-zephyr-eabi\bin\arm-zephyr-eabi-gdb.exe"
    if (-not (Test-Path -LiteralPath $openOcd -PathType Leaf)) { throw "SDK OpenOCD is missing: $openOcd" }
    if (-not (Test-Path -LiteralPath $gdb -PathType Leaf)) { throw "SDK ARM GDB is missing: $gdb" }
    return @{ West = $westPath; OpenOcd = $openOcd; Gdb = $gdb }
}

switch ($Action) {
    "build" {
        $target = Resolve-ZephyrProject
        if ([string]::IsNullOrWhiteSpace($BuildRoot)) {
            $BuildRoot = Join-Path $RepoRoot "local\build"
        }
        $localVenv = Join-Path $RepoRoot "local\cache\zephyrproject\.venv\Scripts"
        $westPath = Resolve-Tool -Value $West -Name "West" -PreferredPaths @(Join-Path $localVenv "west.exe")
        $ninjaPath = Resolve-Tool -Value $Ninja -Name "Ninja" -PreferredPaths @(Join-Path $localVenv "ninja.exe")
        if ($Pristine) {
            & (Join-Path $RepoRoot "tools\build-matrix.ps1") -Target $target -BuildRoot $BuildRoot -West $westPath -Ninja $ninjaPath -Jobs $Jobs -Pristine
        }
        else {
            & (Join-Path $RepoRoot "tools\build-matrix.ps1") -Target $target -BuildRoot $BuildRoot -West $westPath -Ninja $ninjaPath -Jobs $Jobs
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

    "flash" {
        $buildDir = Get-ExistingZephyrBuild
        $tools = Get-OpenOcdTools
        Write-Host "Flashing $Project from $buildDir with OpenOCD, then verifying the written image."
        & $tools.West flash -d $buildDir -r openocd --no-rebuild --gdb $tools.Gdb --openocd $tools.OpenOcd -- --no-erase --verify
        exit $LASTEXITCODE
    }

    "debug" {
        $buildDir = Get-ExistingZephyrBuild
        $tools = Get-OpenOcdTools
        Write-Host "Starting OpenOCD and SDK ARM GDB for $Project from $buildDir."
        & $tools.West debug -d $buildDir -r openocd --no-rebuild --gdb $tools.Gdb --openocd $tools.OpenOcd -- --no-load
        exit $LASTEXITCODE
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
            $projects = @($ProjectMap.Keys)
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
