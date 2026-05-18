[CmdletBinding(SupportsShouldProcess = $true)]
param(
    [switch]$Deep,
    [switch]$Quiet
)

$ErrorActionPreference = 'Stop'
$RepoRoot = Split-Path -Parent $MyInvocation.MyCommand.Path

$roots = @(
    Join-Path $RepoRoot 'projects'
    Join-Path $RepoRoot 'boards'
) | Where-Object { Test-Path -LiteralPath $_ }

$defaultFilePatterns = @(
    '*.bak',
    '*.lst',
    '*.lnp',
    '*.obj',
    '*.tmp',
    '*.TMP',
    '*.__i',
    '*.crf',
    '*.o',
    '*.d',
    '*.axf',
    '*.dep',
    '*.htm',
    '*.hsc',
    '*.map',
    '*.hex',
    '*.bin',
    '*.tra',
    '*.build.log',
    'codex_build*.log',
    '*JLinkLog.txt'
)

$deepFilePatterns = @(
    '*.uvopt',
    '*.uvoptx',
    '*.uvoptx.*',
    '*uvgui*',
    '*.scvd',
    '*.iex',
    '*.ini'
)

$namedBuildDirs = @(
    'Objects',
    'Listings',
    'RTE',
    'DebugConfig',
    'standard_tpye_c'
)

if ($Deep) {
    $namedBuildDirs += @('Debug', 'Release', 'build')
}

$paths = [ordered]@{}

function Add-Candidate {
    param([string]$Path)
    if ([string]::IsNullOrWhiteSpace($Path)) {
        return
    }
    $resolved = Resolve-Path -LiteralPath $Path -ErrorAction SilentlyContinue
    if ($null -eq $resolved) {
        return
    }
    $paths[$resolved.ProviderPath] = $true
}

function Test-AnyPattern {
    param(
        [string]$Name,
        [string[]]$Patterns
    )
    foreach ($pattern in $Patterns) {
        if ($Name -like $pattern) {
            return $true
        }
    }
    return $false
}

foreach ($root in $roots) {
    Get-ChildItem -LiteralPath $root -Recurse -Force -Directory -ErrorAction SilentlyContinue |
        Where-Object { $namedBuildDirs -contains $_.Name } |
        ForEach-Object { Add-Candidate $_.FullName }

    Get-ChildItem -Path (Join-Path $root '*\MDK-ARM') -Directory -ErrorAction SilentlyContinue |
        ForEach-Object {
            Get-ChildItem -LiteralPath $_.FullName -Force -Directory -ErrorAction SilentlyContinue |
                ForEach-Object { Add-Candidate $_.FullName }
        }

    $patterns = @($defaultFilePatterns)
    if ($Deep) {
        $patterns += $deepFilePatterns
    }

    Get-ChildItem -LiteralPath $root -Recurse -Force -File -ErrorAction SilentlyContinue |
        Where-Object { Test-AnyPattern $_.Name $patterns } |
        ForEach-Object { Add-Candidate $_.FullName }
}

$items = $paths.Keys | Sort-Object
$totalBytes = 0L
foreach ($item in $items) {
    if (Test-Path -LiteralPath $item -PathType Container) {
        $sum = (Get-ChildItem -LiteralPath $item -Recurse -Force -File -ErrorAction SilentlyContinue |
                Measure-Object -Property Length -Sum).Sum
        if ($null -ne $sum) {
            $totalBytes += [int64]$sum
        }
    } elseif (Test-Path -LiteralPath $item -PathType Leaf) {
        $totalBytes += (Get-Item -LiteralPath $item -Force).Length
    }
}

foreach ($item in $items) {
    if ($PSCmdlet.ShouldProcess($item, 'Remove build output')) {
        Remove-Item -LiteralPath $item -Recurse -Force -ErrorAction SilentlyContinue
    }
}

if (-not $Quiet) {
    $mb = if ($totalBytes -gt 0) { $totalBytes / 1MB } else { 0 }
    '{0} item(s), about {1:N2} MB matched.' -f $items.Count, $mb
    if (-not $Deep) {
        'Use -Deep to also remove Keil local option/window files such as *.uvoptx and *uvgui*.'
    }
}
