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

function Get-ProfileModules {
    param([string]$Content)

    $match = [regex]::Match($Content, '\.task_modules\s*=\s*\{(?<body>.*?)\}', [System.Text.RegularExpressions.RegexOptions]::Singleline)
    if (-not $match.Success) {
        return @()
    }

    return @([regex]::Matches($match.Groups["body"].Value, 'ROBOT_TASK_MODULE_[A-Z0-9_]+') |
        ForEach-Object { $_.Value } |
        Select-Object -Unique)
}

function Get-ProfileModuleCount {
    param([string]$Content)

    $match = [regex]::Match($Content, '\.task_module_count\s*=\s*(\d+)u?')
    if (-not $match.Success) {
        return $null
    }

    return [int]$match.Groups[1].Value
}

function Get-TaskModuleEnums {
    param([string]$Content)

    return @([regex]::Matches($Content, 'ROBOT_TASK_MODULE_[A-Z0-9_]+\s*=') |
        ForEach-Object { $_.Value.TrimEnd("=", " ") } |
        Where-Object { $_ -ne "ROBOT_TASK_MODULE_NONE" } |
        Select-Object -Unique)
}

function Get-NamedTaskModules {
    param([string]$Content)

    return @([regex]::Matches($Content, '\{\s*(ROBOT_TASK_MODULE_[A-Z0-9_]+)\s*,\s*"task\.[^"]+"\s*\}') |
        ForEach-Object { $_.Groups[1].Value } |
        Select-Object -Unique)
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
    $configHeader = Get-Content -LiteralPath $configH -Raw
    $moduleCount = Get-ProfileModuleCount $configContent
    $modules = @(Get-ProfileModules $configContent)

    $profileFamilyPattern = '\.(locomotion_family|gimbal_family|arm_family)\s*=|LOCOMOTION_FAMILY_|GIMBAL_FAMILY_|ARM_FAMILY_'
    if ($configContent -match $profileFamilyPattern -or $configHeader -match $profileFamilyPattern) {
        Add-CheckError "$(Format-RepoPath $configC): profile task selection must use task_modules only; family fields are no longer accepted."
    }
    if ($configContent -notmatch '\.task_module_count\s*=') {
        Add-CheckError "$(Format-RepoPath $configC): cannot find .task_module_count; task creation now uses the explicit module list."
    }
    if ($modules.Count -eq 0) {
        Add-CheckError "$(Format-RepoPath $configC): cannot find any ROBOT_TASK_MODULE_* entries."
    }
    if ($null -ne $moduleCount -and $moduleCount -ne $modules.Count) {
        Add-CheckError "$(Format-RepoPath $configC): .task_module_count is $moduleCount, but task_modules contains $($modules.Count) unique modules."
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

    Test-ProfileTaskMapping $Project $modules $sourceSet $taskText
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

function Test-RequiredTaskTextAny {
    param(
        [object]$Project,
        [string]$TaskText,
        [string[]]$Needles
    )

    foreach ($needle in $Needles) {
        if ($TaskText -match [regex]::Escape($needle)) {
            return
        }
    }

    $needleList = $Needles -join "', '"
    Add-CheckError "$(Format-RepoPath $Project.UvprojxPath): task creation source does not mention any of '$needleList'."
}

function Test-ProfileHasModule {
    param(
        [string[]]$Modules,
        [string]$Module
    )

    return $Modules -contains $Module
}

function Test-ProfileTaskMapping {
    param(
        [object]$Project,
        [string[]]$Modules,
        [object]$SourceSet,
        [string]$TaskText
    )

    if (Test-ProfileHasModule $Modules "ROBOT_TASK_MODULE_CLASSIC_CHASSIS") {
        Test-RequiredSource $Project $SourceSet "shared\application\chassis\chassis_control_task.c"
        Test-RequiredTaskText $Project $TaskText "ROBOT_TASK_MODULE_CLASSIC_CHASSIS"
        Test-RequiredTaskText $Project $TaskText "chassis_control_task"
    }
    if (Test-ProfileHasModule $Modules "ROBOT_TASK_MODULE_WHEELLEG_MIT") {
        Test-RequiredSource $Project $SourceSet "shared\application\wheelleg\wheelleg_mit_task.c"
        Test-RequiredTaskText $Project $TaskText "ROBOT_TASK_MODULE_WHEELLEG_MIT"
        Test-RequiredTaskText $Project $TaskText "wheelleg_mit_task"
    }
    if (Test-ProfileHasModule $Modules "ROBOT_TASK_MODULE_WHEELLEG_SERVO") {
        Add-CheckError "$(Format-RepoPath $Project.UvprojxPath): profile lists WHEELLEG_SERVO, but no servo wheel-leg task is wired yet."
    }
    if (Test-ProfileHasModule $Modules "ROBOT_TASK_MODULE_SINGLE_GIMBAL") {
        Test-RequiredSource $Project $SourceSet "shared\application\gimbal\gimbal_control_task.c"
        Test-RequiredTaskText $Project $TaskText "ROBOT_TASK_MODULE_SINGLE_GIMBAL"
        Test-RequiredTaskText $Project $TaskText "gimbal_control_task"
    }
    if (Test-ProfileHasModule $Modules "ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL") {
        Test-RequiredSource $Project $SourceSet "shared\application\gimbal\gimbal_control_task.c"
        Test-RequiredTaskText $Project $TaskText "ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL"
        Test-RequiredTaskText $Project $TaskText "dual_yaw_gimbal_control_task"
    }
    if (Test-ProfileHasModule $Modules "ROBOT_TASK_MODULE_ARM") {
        Test-RequiredSource $Project $SourceSet "shared\application\arm\arm_task.c"
        Test-RequiredTaskText $Project $TaskText "ROBOT_TASK_MODULE_ARM"
        Test-RequiredTaskText $Project $TaskText "arm_task"
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

function Test-TaskModuleNames {
    param([object[]]$Projects)

    Write-Host "[check] task module names"

    $profileHeader = Join-Path $script:RepoRoot "shared\application\robot\robot_task_profile.h"
    if (-not (Test-Path -LiteralPath $profileHeader -PathType Leaf)) {
        Add-CheckError "Missing robot task profile header: $(Format-RepoPath $profileHeader)"
        return
    }

    $profileContent = Get-Content -LiteralPath $profileHeader -Raw
    $namedModules = New-Object System.Collections.Generic.HashSet[string]
    foreach ($module in Get-NamedTaskModules $profileContent) {
        [void]$namedModules.Add($module)
    }

    foreach ($project in $Projects) {
        $configHeader = Join-Path $script:RepoRoot "Robotconfig\$($project.Name)\config.h"
        if (-not (Test-Path -LiteralPath $configHeader -PathType Leaf)) {
            continue
        }

        $configContent = Get-Content -LiteralPath $configHeader -Raw
        foreach ($module in Get-TaskModuleEnums $configContent) {
            if (-not $namedModules.Contains($module)) {
                Add-CheckError "$(Format-RepoPath $configHeader): $module has no task.* name in $(Format-RepoPath $profileHeader)."
            }
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
Test-TaskModuleNames $projects
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
