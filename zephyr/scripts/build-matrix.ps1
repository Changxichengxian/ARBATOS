[CmdletBinding()]
param(
    [ValidateSet(
        'hero-c',
        'hero-m',
        'infantry-a',
        'sentinel-m',
        'carrier-a',
        'miniwheeleg-m',
        'miniwheeleg-c'
    )]
    [string[]] $Target = @(
        'hero-c',
        'hero-m',
        'infantry-a',
        'sentinel-m',
        'carrier-a',
        'miniwheeleg-m',
        'miniwheeleg-c'
    ),

    [switch] $Pristine,

    [string] $West = 'west',

    [string] $Ninja
)

$ErrorActionPreference = 'Stop'
$appRoot = Split-Path -Parent $PSScriptRoot

$targets = @{
    'hero-c'        = @{ Board = 'dji_c_f407'; Config = 'hero-c.conf' }
    'miniwheeleg-c' = @{ Board = 'dji_c_f407'; Config = 'miniwheeleg-c.conf' }
    'infantry-a'    = @{ Board = 'dji_a_f427'; Config = 'infantry-a.conf' }
    'carrier-a'     = @{ Board = 'dji_a_f427'; Config = 'carrier-a.conf' }
    'hero-m'        = @{ Board = 'dm_mc02_h7'; Config = 'hero-m.conf' }
    'miniwheeleg-m' = @{ Board = 'dm_mc02_h7'; Config = 'miniwheeleg-m.conf' }
    'sentinel-m'    = @{
        Board = 'dm_mc02_h7'
        Config = 'sentinel-m.conf'
        Overlay = 'sentinel-m.overlay'
    }
}

foreach ($name in $Target) {
    $entry = $targets[$name]
    $buildDir = Join-Path $appRoot "build-$name"
    $configPath = Join-Path (Join-Path $appRoot 'targets') $entry.Config
    $westArgs = @(
        'build',
        '-s', $appRoot,
        '-d', $buildDir,
        '-b', $entry.Board
    )

    if ($Pristine) {
        $westArgs += @('-p', 'always')
    }

    $configCmakePath = $configPath -replace '\\', '/'
    $cmakeArgs = @("-DEXTRA_CONF_FILE=$configCmakePath")
    if ($Ninja) {
        $cmakeArgs += "-DCMAKE_MAKE_PROGRAM=$($Ninja -replace '\\', '/')"
    }
    if ($entry.ContainsKey('Overlay')) {
        $overlayPath = Join-Path (Join-Path $appRoot 'targets') $entry.Overlay
        $overlayCmakePath = $overlayPath -replace '\\', '/'
        $cmakeArgs += "-DDTC_OVERLAY_FILE=$overlayCmakePath"
    }
    $westArgs += '--'
    $westArgs += $cmakeArgs

    Write-Host "==> Building $name ($($entry.Board))"
    & $West @westArgs
    if ($LASTEXITCODE -ne 0) {
        throw "Target $name failed with exit code $LASTEXITCODE"
    }
}
