[CmdletBinding()]
param(
    [string[]] $Target = @('all'),

    [switch] $Pristine,

    [string] $BuildRoot,

    [string] $West = 'west',

    [string] $Ninja,

    [ValidateRange(1, 64)]
    [int] $Jobs = 2
)

$ErrorActionPreference = 'Stop'
# 限制本次编译的并行数，避免默认占满电脑。
$env:CMAKE_BUILD_PARALLEL_LEVEL = [string]$Jobs
$repoRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$appRoot = Join-Path $repoRoot 'projects'
if ([string]::IsNullOrWhiteSpace($BuildRoot)) {
    $BuildRoot = Join-Path $repoRoot 'local\\build'
}
$buildRootPath = [System.IO.Path]::GetFullPath($BuildRoot)

# Keep this command usable from a CLion External Tool or a plain shell. These
# assignments affect only this PowerShell process and prefer an explicit user
# environment when one is already configured.
$localZephyrBase = Join-Path $repoRoot 'local\cache\zephyrproject\zephyr'
if (-not $env:ZEPHYR_BASE -and (Test-Path -LiteralPath $localZephyrBase -PathType Container)) {
    $env:ZEPHYR_BASE = $localZephyrBase
}
$localZephyrSdk = Join-Path $repoRoot 'local\cache\zephyr-sdk'
if (-not $env:ZEPHYR_SDK_INSTALL_DIR -and (Test-Path -LiteralPath $localZephyrSdk -PathType Container)) {
    $env:ZEPHYR_SDK_INSTALL_DIR = $localZephyrSdk
}
if (-not $env:Zephyr_sdk_DIR -and $env:ZEPHYR_SDK_INSTALL_DIR) {
    $env:Zephyr_sdk_DIR = Join-Path $env:ZEPHYR_SDK_INSTALL_DIR 'cmake'
}
$localVenvScripts = Join-Path $repoRoot 'local\cache\zephyrproject\.venv\Scripts'
if ((Test-Path -LiteralPath $localVenvScripts -PathType Container) -and
    (($env:Path -split ';') -notcontains $localVenvScripts)) {
    $env:Path = "$localVenvScripts;$env:Path"
}

function Get-CommandPath {
    param([string] $Value, [string] $Name)

    if (Test-Path -LiteralPath $Value -PathType Leaf) {
        return [System.IO.Path]::GetFullPath($Value)
    }
    $command = Get-Command $Value -ErrorAction SilentlyContinue
    if ($null -eq $command) {
        throw "Cannot find ${Name}: $Value. Install it or provide its executable path."
    }
    return $command.Source
}

function Get-RobotTargets {
    $generator = Join-Path $repoRoot 'tools\config\RobotConfigGen.py'
    if (-not (Test-Path -LiteralPath $generator -PathType Leaf)) {
        throw "Cannot find robot configuration generator: $generator"
    }
    $python = Get-CommandPath -Value 'python' -Name 'Python'
    $json = & $python $generator list --json
    if ($LASTEXITCODE -ne 0) {
        throw 'Robot configuration generator failed while listing targets.'
    }
    $items = @($json | ConvertFrom-Json)
    if ($items.Count -eq 0) {
        throw 'No RobotConfig.toml target is available under Robotconfig.'
    }
    return $items
}

function Move-PreviousBuild {
    param([string] $TargetName, [string] $BuildDir)

    $buildDirPath = [System.IO.Path]::GetFullPath($BuildDir)
    $backupRoot = [System.IO.Path]::GetFullPath((Join-Path $buildRootPath '.pristine-backups'))
    $rootPrefix = $buildRootPath.TrimEnd('\', '/') + [System.IO.Path]::DirectorySeparatorChar
    foreach ($pathToCheck in @($buildDirPath, $backupRoot)) {
        if (-not $pathToCheck.StartsWith($rootPrefix, [System.StringComparison]::OrdinalIgnoreCase)) {
            throw "Unsafe pristine path outside BuildRoot: $pathToCheck"
        }
    }
    foreach ($pathToCheck in @($buildRootPath, $buildDirPath, $backupRoot)) {
        if (Test-Path -LiteralPath $pathToCheck) {
            $item = Get-Item -LiteralPath $pathToCheck -Force
            if (($item.Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0) {
                throw "Refusing pristine through a symbolic link or junction: $pathToCheck"
            }
        }
    }

    if (-not (Test-Path -LiteralPath $buildDirPath)) {
        return
    }
    New-Item -ItemType Directory -Path $backupRoot -Force | Out-Null
    $stamp = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
    $backupDir = Join-Path $backupRoot ("{0}-{1}" -f $TargetName, $stamp)
    $suffix = 1
    while (Test-Path -LiteralPath $backupDir) {
        $backupDir = Join-Path $backupRoot ("{0}-{1}-{2}" -f $TargetName, $stamp, $suffix)
        $suffix++
    }
    $backupDir = [System.IO.Path]::GetFullPath($backupDir)
    if (-not $backupDir.StartsWith($rootPrefix, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Unsafe pristine backup path outside BuildRoot: $backupDir"
    }
    Write-Host "[pristine] preserving previous output: $buildDirPath -> $backupDir"
    try {
        Move-Item -LiteralPath $buildDirPath -Destination $backupDir -ErrorAction Stop
    }
    catch {
        throw "Cannot preserve $buildDirPath because a file is in use. Close the CMake or Ninja process and retry -Pristine; no build output was deleted. Details: $($_.Exception.Message)"
    }
}

$westPath = Get-CommandPath -Value $West -Name 'West'
if ($Ninja) {
    $Ninja = Get-CommandPath -Value $Ninja -Name 'Ninja'
}

$targets = @(Get-RobotTargets)
$targetsByPreset = @{}
foreach ($entry in $targets) {
    if ([string]::IsNullOrWhiteSpace($entry.preset) -or [string]::IsNullOrWhiteSpace($entry.name)) {
        throw 'Robot configuration list contains a target without name or preset.'
    }
    $targetsByPreset[$entry.preset.ToLowerInvariant()] = $entry
    $targetsByPreset[$entry.name.ToLowerInvariant()] = $entry
}

$selectedTargets = New-Object System.Collections.Generic.List[object]
foreach ($requestedName in $Target) {
    $requestedName = $requestedName.ToLowerInvariant()
    if ($requestedName -eq 'all') {
        foreach ($entry in $targets) {
            if (-not (@($selectedTargets | Where-Object { $_.name -eq $entry.name }).Count)) { $selectedTargets.Add($entry) }
        }
    }
    elseif (-not $targetsByPreset.ContainsKey($requestedName)) {
        throw "Unknown target: $requestedName. Use all or one of: $($targets.name -join ', ')"
    }
    else {
        $entry = $targetsByPreset[$requestedName]
        if (-not (@($selectedTargets | Where-Object { $_.name -eq $entry.name }).Count)) { $selectedTargets.Add($entry) }
    }
}

foreach ($entry in $selectedTargets) {
    $name = $entry.preset
    $buildDir = Join-Path $buildRootPath $name
    if ($Pristine) {
        Move-PreviousBuild -TargetName $name -BuildDir $buildDir
    }
    $westArgs = @(
        'build',
        '-s', $appRoot,
        '-d', $buildDir,
        '-b', $entry.board,
        '-p', 'never'
    )

    $cmakeArgs = @("-DARBATOS_ROBOT=$($entry.name)")
    if ($Ninja) {
        $cmakeArgs += "-DCMAKE_MAKE_PROGRAM=$($Ninja -replace '\\', '/')"
    }
    $westArgs += '--'
    $westArgs += $cmakeArgs

    Write-Host "==> Building $($entry.name) ($($entry.board)) -> $buildDir"
    & $westPath @westArgs
    if ($LASTEXITCODE -ne 0) {
        throw "Target $name failed with exit code $LASTEXITCODE"
    }
}
