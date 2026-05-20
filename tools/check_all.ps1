param(
    [switch]$AllText
)

$ErrorActionPreference = "Stop"

$RepoRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot ".."))
$Errors = New-Object System.Collections.Generic.List[string]
$Warnings = New-Object System.Collections.Generic.List[string]

function Add-CheckError {
    param([string]$Message)
    $script:Errors.Add($Message)
}

function Add-CheckWarning {
    param([string]$Message)
    $script:Warnings.Add($Message)
}

function Format-RepoPath {
    param([string]$Path)

    $fullPath = [System.IO.Path]::GetFullPath($Path)
    if ($fullPath.StartsWith($script:RepoRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
        return $fullPath.Substring($script:RepoRoot.Length).TrimStart("\", "/")
    }

    return $fullPath
}

function Resolve-ProjectPath {
    param(
        [string]$ProjectDir,
        [string]$Path
    )

    $trimmed = $Path.Trim()
    if ([System.IO.Path]::IsPathRooted($trimmed)) {
        return [System.IO.Path]::GetFullPath($trimmed)
    }

    return [System.IO.Path]::GetFullPath((Join-Path $ProjectDir $trimmed))
}

function Get-UvProjectInfo {
    $projectRoot = Join-Path $script:RepoRoot "projects"
    $projectFiles = @(Get-ChildItem -Path $projectRoot -Recurse -Filter "*.uvprojx" | Sort-Object FullName)

    foreach ($projectFile in $projectFiles) {
        $projectDir = Split-Path -Parent $projectFile.FullName
        $projectName = Split-Path -Leaf (Split-Path -Parent $projectDir)

        try {
            $xml = [xml](Get-Content -LiteralPath $projectFile.FullName -Raw)
        }
        catch {
            Add-CheckError "Cannot parse XML: $(Format-RepoPath $projectFile.FullName) ($($_.Exception.Message))"
            continue
        }

        $filePaths = @($xml.SelectNodes("//FilePath") | ForEach-Object { $_.InnerText } | Where-Object { -not [string]::IsNullOrWhiteSpace($_) })
        $includePaths = @($xml.SelectNodes("//IncludePath") | ForEach-Object { $_.InnerText -split ";" } | Where-Object { -not [string]::IsNullOrWhiteSpace($_) })
        $targetNames = @($xml.SelectNodes("//TargetName") | ForEach-Object { $_.InnerText } | Where-Object { -not [string]::IsNullOrWhiteSpace($_) })
        $packIds = @($xml.SelectNodes("//PackID") | ForEach-Object { $_.InnerText } | Where-Object { -not [string]::IsNullOrWhiteSpace($_) })
        $compilerIds = @($xml.SelectNodes("//pCCUsed") | ForEach-Object { $_.InnerText } | Where-Object { -not [string]::IsNullOrWhiteSpace($_) })

        $resolvedFiles = @()
        foreach ($path in $filePaths) {
            if ($path.Trim() -match '^\$\$') {
                continue
            }
            $resolvedFiles += Resolve-ProjectPath $projectDir $path
        }

        [pscustomobject]@{
            Name = $projectName
            UvprojxPath = $projectFile.FullName
            ProjectDir = $projectDir
            Xml = $xml
            FilePaths = $filePaths
            IncludePaths = $includePaths
            TargetNames = $targetNames
            PackIds = $packIds
            CompilerIds = $compilerIds
            ResolvedFiles = $resolvedFiles
        }
    }
}

function Get-ProfileValue {
    param(
        [string]$Content,
        [string]$FieldName
    )

    $match = [regex]::Match($Content, "\.$FieldName\s*=\s*([A-Z0-9_]+)")
    if (-not $match.Success) {
        return $null
    }

    return $match.Groups[1].Value
}

function Test-UvProject {
    param([object]$Project)

    Write-Host "[check] project $($Project.Name)"

    foreach ($targetName in $Project.TargetNames) {
        if ($targetName -ne $Project.Name) {
            Add-CheckWarning "$(Format-RepoPath $Project.UvprojxPath): TargetName '$targetName' differs from folder '$($Project.Name)'."
        }
    }

    if ($Project.PackIds.Count -eq 0) {
        Add-CheckWarning "$(Format-RepoPath $Project.UvprojxPath): no PackID recorded."
    }

    if ($Project.CompilerIds.Count -eq 0) {
        Add-CheckWarning "$(Format-RepoPath $Project.UvprojxPath): no compiler version recorded in pCCUsed."
    }

    $seenFiles = @{}
    foreach ($path in $Project.FilePaths) {
        if ($path.Trim() -match '^\$\$') {
            continue
        }

        $resolved = Resolve-ProjectPath $Project.ProjectDir $path
        if (-not (Test-Path -LiteralPath $resolved -PathType Leaf)) {
            Add-CheckError "$(Format-RepoPath $Project.UvprojxPath): missing source file '$path' => $(Format-RepoPath $resolved)"
            continue
        }

        $key = $resolved.ToLowerInvariant()
        if ($seenFiles.ContainsKey($key)) {
            Add-CheckWarning "$(Format-RepoPath $Project.UvprojxPath): duplicate source file '$path'."
        }
        else {
            $seenFiles[$key] = $true
        }
    }

    $seenIncludePaths = New-Object System.Collections.Generic.HashSet[string]
    foreach ($includePath in $Project.IncludePaths) {
        $trimmed = $includePath.Trim()
        if ($trimmed -match '^\$\$|^\$\(|^%') {
            continue
        }

        $includeKey = $trimmed.Replace("/", "\").ToLowerInvariant()
        if (-not $seenIncludePaths.Add($includeKey)) {
            continue
        }

        $resolved = Resolve-ProjectPath $Project.ProjectDir $trimmed
        if (-not (Test-Path -LiteralPath $resolved -PathType Container)) {
            Add-CheckWarning "$(Format-RepoPath $Project.UvprojxPath): include path not found locally '$trimmed' => $(Format-RepoPath $resolved)"
        }
    }

    $robotConfigs = New-Object System.Collections.Generic.HashSet[string]
    $boards = New-Object System.Collections.Generic.HashSet[string]
    foreach ($path in ($Project.FilePaths + $Project.IncludePaths)) {
        foreach ($match in [regex]::Matches($path, 'Robotconfig[\\/]+([^\\/;]+)')) {
            [void]$robotConfigs.Add($match.Groups[1].Value)
        }
        foreach ($match in [regex]::Matches($path, 'boards[\\/]+([^\\/;]+)')) {
            [void]$boards.Add($match.Groups[1].Value)
        }
    }

    if (-not $robotConfigs.Contains($Project.Name)) {
        Add-CheckError "$(Format-RepoPath $Project.UvprojxPath): does not reference Robotconfig/$($Project.Name)."
    }

    foreach ($robotConfig in $robotConfigs) {
        if ($robotConfig -ne $Project.Name) {
            Add-CheckError "$(Format-RepoPath $Project.UvprojxPath): also references Robotconfig/$robotConfig."
        }
    }

    if ($boards.Count -eq 0) {
        Add-CheckWarning "$(Format-RepoPath $Project.UvprojxPath): no boards/<BOARD> path found."
    }

    $configDir = Join-Path $script:RepoRoot "Robotconfig\$($Project.Name)"
    $configC = Join-Path $configDir "config.c"
    $configH = Join-Path $configDir "config.h"

    if (-not (Test-Path -LiteralPath $configC -PathType Leaf)) {
        Add-CheckError "Missing Robotconfig config.c for $($Project.Name): $(Format-RepoPath $configC)"
        return
    }

    if (-not (Test-Path -LiteralPath $configH -PathType Leaf)) {
        Add-CheckError "Missing Robotconfig config.h for $($Project.Name): $(Format-RepoPath $configH)"
        return
    }

    $configContent = Get-Content -LiteralPath $configC -Raw
    $locomotion = Get-ProfileValue $configContent "locomotion_family"
    $gimbal = Get-ProfileValue $configContent "gimbal_family"
    $arm = Get-ProfileValue $configContent "arm_family"

    if ($null -eq $locomotion) {
        Add-CheckError "$(Format-RepoPath $configC): cannot find .locomotion_family."
    }
    if ($null -eq $gimbal) {
        Add-CheckError "$(Format-RepoPath $configC): cannot find .gimbal_family."
    }
    if ($null -eq $arm) {
        Add-CheckError "$(Format-RepoPath $configC): cannot find .arm_family."
    }

    $sourceSet = New-Object System.Collections.Generic.HashSet[string]
    foreach ($file in $Project.ResolvedFiles) {
        [void]$sourceSet.Add((Format-RepoPath $file).Replace("/", "\").ToLowerInvariant())
    }

    $taskSourceFiles = @($Project.ResolvedFiles | Where-Object { (Split-Path -Leaf $_) -match 'freertos\.c$' })
    $taskText = ""
    foreach ($taskSource in $taskSourceFiles) {
        if (Test-Path -LiteralPath $taskSource -PathType Leaf) {
            $taskText += "`n" + (Get-Content -LiteralPath $taskSource -Raw)
        }
    }

    Test-ProfileTaskMapping $Project $locomotion $gimbal $arm $sourceSet $taskText
}

function Test-RequiredSource {
    param(
        [object]$Project,
        [object]$SourceSet,
        [string]$RepoRelativePath
    )

    $key = $RepoRelativePath.Replace("/", "\").ToLowerInvariant()
    if (-not $SourceSet.Contains($key)) {
        Add-CheckError "$(Format-RepoPath $Project.UvprojxPath): profile needs '$RepoRelativePath', but the project does not include it."
    }
}

function Test-RequiredTaskText {
    param(
        [object]$Project,
        [string]$TaskText,
        [string]$Needle
    )

    if ($TaskText -notmatch [regex]::Escape($Needle)) {
        Add-CheckError "$(Format-RepoPath $Project.UvprojxPath): task creation source does not mention '$Needle'."
    }
}

function Test-ProfileTaskMapping {
    param(
        [object]$Project,
        [string]$Locomotion,
        [string]$Gimbal,
        [string]$Arm,
        [object]$SourceSet,
        [string]$TaskText
    )

    switch ($Locomotion) {
        "LOCOMOTION_FAMILY_CLASSIC_CHASSIS" {
            Test-RequiredSource $Project $SourceSet "shared\application\chassis\chassis_control_task.c"
            Test-RequiredTaskText $Project $TaskText "robot_profile_need_classic_chassis_control_task"
            Test-RequiredTaskText $Project $TaskText "chassis_control_task"
        }
        "LOCOMOTION_FAMILY_WHEELLEG_MIT" {
            Test-RequiredSource $Project $SourceSet "shared\application\wheelleg\wheelleg_mit_task.c"
            Test-RequiredTaskText $Project $TaskText "robot_profile_is_wheelleg_mit"
            Test-RequiredTaskText $Project $TaskText "wheelleg_mit_task"
        }
        "LOCOMOTION_FAMILY_WHEELLEG_SERVO" {
            Add-CheckError "$(Format-RepoPath $Project.UvprojxPath): profile selects WHEELLEG_SERVO, but no servo wheel-leg task is wired yet."
        }
        "LOCOMOTION_FAMILY_NONE" {}
        $null {}
        default {
            Add-CheckError "$(Format-RepoPath $Project.UvprojxPath): unknown locomotion family '$Locomotion'."
        }
    }

    switch ($Gimbal) {
        "GIMBAL_FAMILY_SINGLE" {
            Test-RequiredSource $Project $SourceSet "shared\application\gimbal\gimbal_control_task.c"
            Test-RequiredTaskText $Project $TaskText "robot_profile_need_single_gimbal_control_task"
            Test-RequiredTaskText $Project $TaskText "gimbal_control_task"
        }
        "GIMBAL_FAMILY_DUAL" {
            Add-CheckError "$(Format-RepoPath $Project.UvprojxPath): profile selects GIMBAL_DUAL, but no dual-gimbal task is wired yet."
        }
        "GIMBAL_FAMILY_NONE" {}
        $null {}
        default {
            Add-CheckError "$(Format-RepoPath $Project.UvprojxPath): unknown gimbal family '$Gimbal'."
        }
    }

    switch ($Arm) {
        "ARM_FAMILY_UNIFIED" {
            Test-RequiredSource $Project $SourceSet "shared\application\arm\arm_task.c"
            Test-RequiredTaskText $Project $TaskText "robot_profile_need_arm_task"
            Test-RequiredTaskText $Project $TaskText "arm_task"
        }
        "ARM_FAMILY_NONE" {}
        $null {}
        default {
            Add-CheckError "$(Format-RepoPath $Project.UvprojxPath): unknown arm family '$Arm'."
        }
    }
}

function Test-RobotconfigCoverage {
    param([object[]]$Projects)

    Write-Host "[check] Robotconfig coverage"

    $projectNames = New-Object System.Collections.Generic.HashSet[string]
    foreach ($project in $Projects) {
        [void]$projectNames.Add($project.Name)
    }

    $robotConfigDirs = @(Get-ChildItem -Path (Join-Path $script:RepoRoot "Robotconfig") -Directory | Sort-Object Name)
    foreach ($dir in $robotConfigDirs) {
        if (-not $projectNames.Contains($dir.Name)) {
            Add-CheckWarning "Robotconfig/$($dir.Name) has no matching projects/$($dir.Name) entry."
        }
    }
}

function Test-PythonTools {
    Write-Host "[check] python tool syntax"

    $pythonFiles = @(Get-ChildItem -Path (Join-Path $script:RepoRoot "tools") -Recurse -Filter "*.py" | Sort-Object FullName)
    if ($pythonFiles.Count -eq 0) {
        return
    }

    $python = Get-Command python -ErrorAction SilentlyContinue
    if ($null -eq $python) {
        Add-CheckWarning "python is not available; skipped Python tool syntax checks."
        return
    }

    $checkCode = 'import ast, sys, tokenize; p=sys.argv[1]; f=tokenize.open(p); src=f.read(); f.close(); ast.parse(src, filename=p)'
    foreach ($pythonFile in $pythonFiles) {
        $output = & $python.Source -c $checkCode $pythonFile.FullName 2>&1
        if ($LASTEXITCODE -ne 0) {
            Add-CheckError "$(Format-RepoPath $pythonFile.FullName): Python syntax check failed: $output"
        }
    }
}

function Get-TextFilesToCheck {
    if (-not $AllText) {
        try {
            $files = @(git -C $script:RepoRoot ls-files 2>$null)
            if ($LASTEXITCODE -eq 0 -and $files.Count -gt 0) {
                return @($files | Where-Object { $_ -match '\.(md|txt|yml|yaml|ps1|cmd)$' })
            }
        }
        catch {
        }
    }

    return @(Get-ChildItem -Path $script:RepoRoot -Recurse -File |
        Where-Object {
            $_.FullName -notmatch '\\\.git\\' -and
            $_.FullName -notmatch '\\local\\reference\\' -and
            $_.Extension -match '^\.(md|txt|yml|yaml|ps1|cmd)$'
        } |
        ForEach-Object { Format-RepoPath $_.FullName })
}

function Test-StaleText {
    Write-Host "[check] stale text"

    $stalePatterns = @(
        [pscustomobject]@{
            Pattern = 'docs/legal/(CONTRIBUTING|CLA)\.md'
            Message = 'legal document path should be legal/<file>, not docs/legal/<file>.'
        },
        [pscustomobject]@{
            Pattern = 'MIT wheel-leg.*no actual|wheel-leg MIT.*not wired|wheelleg_mit_task.*not wired'
            Message = 'MIT wheel-leg task is already wired; this sentence is stale.'
        },
        [pscustomobject]@{
            Pattern = 'wheelleg_mit_task.*missing'
            Message = 'wheelleg_mit_task already exists and is wired in current entries.'
        }
    )

    foreach ($repoPath in Get-TextFilesToCheck) {
        if ($repoPath.Replace("/", "\") -eq "tools\check_all.ps1") {
            continue
        }

        $fullPath = Join-Path $script:RepoRoot $repoPath
        if (-not (Test-Path -LiteralPath $fullPath -PathType Leaf)) {
            continue
        }

        $content = Get-Content -LiteralPath $fullPath -Raw
        foreach ($stalePattern in $stalePatterns) {
            if ($content -match $stalePattern.Pattern) {
                Add-CheckError "${repoPath}: $($stalePattern.Message)"
            }
        }
    }
}

Write-Host "ARBATOS local checks"
Write-Host "repo: $RepoRoot"

$projects = @(Get-UvProjectInfo)
if ($projects.Count -eq 0) {
    Add-CheckError "No Keil .uvprojx files found under projects/."
}

foreach ($project in $projects) {
    Test-UvProject $project
}

Test-RobotconfigCoverage $projects
Test-PythonTools
Test-StaleText

Write-Host ""
Write-Host "checked projects: $($projects.Count)"
Write-Host "warnings: $($Warnings.Count)"
foreach ($warning in $Warnings) {
    Write-Host "[warn] $warning"
}

Write-Host "errors: $($Errors.Count)"
foreach ($checkError in $Errors) {
    Write-Host "[fail] $checkError"
}

if ($Errors.Count -gt 0) {
    exit 1
}

Write-Host "[ok] checks passed"
