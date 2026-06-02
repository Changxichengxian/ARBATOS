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

    if ($configHeader -notmatch '#include\s+"robot_config_types\.h"') {
        Add-CheckError "$(Format-RepoPath $configH): target config.h must include the shared robot_config_types.h."
    }
    if ($configHeader -match '(?m)^\s*typedef\s+(struct|enum)\b') {
        Add-CheckError "$(Format-RepoPath $configH): target config.h must only carry target identity/build macros; common config types belong in shared\application\robot\robot_config_types.h."
    }

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
    if ($configHeader -match 'robot_device_config_table_t\s+devices\s*;' -and
        $configContent -notmatch '\.devices\s*=') {
        Add-CheckError "$(Format-RepoPath $configC): cannot find .devices initializer; motor_instance now uses the runtime device table."
    }
    if ($configContent -match '\.devices\s*=\s*\{\s*\.count\s*=\s*0u?\s*[,}]') {
        Add-CheckError "$(Format-RepoPath $configC): runtime device table count is 0; motor_instance would have no configured devices."
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

    if ($taskText -notmatch 'app_create_enabled_module_tasks') {
        Add-CheckError "$(Format-RepoPath $Project.UvprojxPath): task creation must use shared app_create_enabled_module_tasks()."
    }
    if ($taskText -notmatch 'robot_control_register_profile_defaults') {
        Add-CheckError "$(Format-RepoPath $Project.UvprojxPath): FreeRTOS init must register default controllers with robot_control_register_profile_defaults()."
    }
    if ($taskText -match 'typedef\s+struct[\s\S]*?app_task_module_desc_t') {
        Add-CheckError "$(Format-RepoPath $Project.UvprojxPath): task source must use app_task_bootstrap.h instead of redefining app_task_module_desc_t locally."
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
    $schemaHeader = Join-Path $script:RepoRoot "shared\application\robot\robot_config_schema.h"
    if (-not (Test-Path -LiteralPath $profileHeader -PathType Leaf)) {
        Add-CheckError "Missing robot task profile header: $(Format-RepoPath $profileHeader)"
        return
    }
    if (-not (Test-Path -LiteralPath $schemaHeader -PathType Leaf)) {
        Add-CheckError "Missing robot config schema header: $(Format-RepoPath $schemaHeader)"
        return
    }

    $profileContent = Get-Content -LiteralPath $profileHeader -Raw
    $schemaContent = Get-Content -LiteralPath $schemaHeader -Raw
    $namedModules = New-Object System.Collections.Generic.HashSet[string]
    foreach ($module in Get-NamedTaskModules $profileContent) {
        [void]$namedModules.Add($module)
    }

    foreach ($module in Get-TaskModuleEnums $schemaContent) {
        if (-not $namedModules.Contains($module)) {
            Add-CheckError "$(Format-RepoPath $schemaHeader): $module has no task.* name in $(Format-RepoPath $profileHeader)."
        }
    }
}

function Test-ProfileIdentity {
    param([object[]]$Projects)

    Write-Host "[check] profile identity"

    $knownProfileKinds = @(
        "ROBOT_PROFILE_KIND_HERO",
        "ROBOT_PROFILE_KIND_INFANTRY",
        "ROBOT_PROFILE_KIND_WHEELLEG",
        "ROBOT_PROFILE_KIND_SENTRY",
        "ROBOT_PROFILE_KIND_CARRIER",
        "ROBOT_PROFILE_KIND_CUSTOM"
    )
    $knownBoardKinds = @(
        "ROBOT_BOARD_KIND_STM32F407",
        "ROBOT_BOARD_KIND_STM32F427",
        "ROBOT_BOARD_KIND_STM32H7",
        "ROBOT_BOARD_KIND_CUSTOM"
    )

    foreach ($project in $Projects) {
        $configHeader = Join-Path $script:RepoRoot "Robotconfig\$($project.Name)\config.h"
        if (-not (Test-Path -LiteralPath $configHeader -PathType Leaf)) {
            continue
        }

        $content = Get-Content -LiteralPath $configHeader -Raw
        foreach ($macro in @(
                "ARBATOS_TARGET_NAME",
                "ARBATOS_BOARD_NAME",
                "ROBOT_PROFILE_KIND",
                "ROBOT_BOARD_KIND",
                "ROBOT_BOARD_CPU_HZ",
                "ROBOT_BOARD_CAN_BUS_COUNT",
                "ROBOT_BOARD_HAS_FPU"
            )) {
            if ($content -notmatch "(?m)^\s*#define\s+$macro\b") {
                Add-CheckError "$(Format-RepoPath $configHeader): missing $macro."
            }
        }

        $profileKind = [regex]::Match($content, '(?m)^\s*#define\s+ROBOT_PROFILE_KIND\s+([A-Z0-9_]+)\b')
        if ($profileKind.Success -and $knownProfileKinds -notcontains $profileKind.Groups[1].Value) {
            Add-CheckError "$(Format-RepoPath $configHeader): unknown ROBOT_PROFILE_KIND '$($profileKind.Groups[1].Value)'."
        }

        $boardKind = [regex]::Match($content, '(?m)^\s*#define\s+ROBOT_BOARD_KIND\s+([A-Z0-9_]+)\b')
        if ($boardKind.Success -and $knownBoardKinds -notcontains $boardKind.Groups[1].Value) {
            Add-CheckError "$(Format-RepoPath $configHeader): unknown ROBOT_BOARD_KIND '$($boardKind.Groups[1].Value)'."
        }

        $cpuHz = [regex]::Match($content, '(?m)^\s*#define\s+ROBOT_BOARD_CPU_HZ\s+(\d+)u?\b')
        if ($cpuHz.Success -and [uint64]$cpuHz.Groups[1].Value -eq 0) {
            Add-CheckError "$(Format-RepoPath $configHeader): ROBOT_BOARD_CPU_HZ must be non-zero."
        }
        $canBusCount = [regex]::Match($content, '(?m)^\s*#define\s+ROBOT_BOARD_CAN_BUS_COUNT\s+(\d+)u?\b')
        if ($canBusCount.Success) {
            $value = [int]$canBusCount.Groups[1].Value
            if ($value -lt 1 -or $value -gt 3) {
                Add-CheckError "$(Format-RepoPath $configHeader): ROBOT_BOARD_CAN_BUS_COUNT '$value' looks invalid."
            }
        }
    }
}

function Test-ProfileProductRules {
    param([object[]]$Projects)

    Write-Host "[check] profile product rules"

    $expectedKinds = @{
        "CARRIER-A" = "ROBOT_PROFILE_KIND_CARRIER"
        "HERO-C" = "ROBOT_PROFILE_KIND_HERO"
        "HERO-M" = "ROBOT_PROFILE_KIND_HERO"
        "INFANTRY-A" = "ROBOT_PROFILE_KIND_INFANTRY"
        "MINIWHEELEG-C" = "ROBOT_PROFILE_KIND_WHEELLEG"
        "MINIWHEELEG-M" = "ROBOT_PROFILE_KIND_WHEELLEG"
        "SENTINEL-A" = "ROBOT_PROFILE_KIND_SENTRY"
    }

    foreach ($project in $Projects) {
        $configHeader = Join-Path $script:RepoRoot "Robotconfig\$($project.Name)\config.h"
        $configC = Join-Path $script:RepoRoot "Robotconfig\$($project.Name)\config.c"
        if (-not (Test-Path -LiteralPath $configHeader -PathType Leaf) -or
            -not (Test-Path -LiteralPath $configC -PathType Leaf)) {
            continue
        }

        $headerContent = Get-Content -LiteralPath $configHeader -Raw
        $configContent = Get-Content -LiteralPath $configC -Raw
        $modules = @(Get-ProfileModules $configContent)

        if ($expectedKinds.ContainsKey($project.Name)) {
            $expectedKind = $expectedKinds[$project.Name]
            if ($headerContent -notmatch "(?m)^\s*#define\s+ROBOT_PROFILE_KIND\s+$expectedKind\b") {
                Add-CheckError "$(Format-RepoPath $configHeader): $($project.Name) should use $expectedKind."
            }
        }

        if ($project.Name -like "HERO-*") {
            foreach ($forbiddenModule in @(
                    "ROBOT_TASK_MODULE_ARM",
                    "ROBOT_TASK_MODULE_WHEELLEG_MIT",
                    "ROBOT_TASK_MODULE_WHEELLEG_SERVO"
                )) {
                if ($modules -contains $forbiddenModule) {
                    Add-CheckError "$(Format-RepoPath $configC): HERO profile must not enable $forbiddenModule."
                }
            }
            foreach ($requiredZero in @(
                    '\.arm\s*=\s*\{\s*0\s*\}',
                    '\.wheelleg_mit\s*=\s*\{\s*0\s*\}',
                    '\.arm_j0_unitree\s*=\s*\{\s*0\s*\}'
                )) {
                if ($configContent -notmatch $requiredZero) {
                    Add-CheckError "$(Format-RepoPath $configC): HERO profile should keep ARM/wheelleg configs zeroed."
                }
            }
        }

        if ($project.Name -like "MINIWHEELEG-*") {
            if ($modules -notcontains "ROBOT_TASK_MODULE_WHEELLEG_MIT") {
                Add-CheckError "$(Format-RepoPath $configC): wheelleg profile should enable ROBOT_TASK_MODULE_WHEELLEG_MIT."
            }
            if ($modules -contains "ROBOT_TASK_MODULE_ARM") {
                Add-CheckError "$(Format-RepoPath $configC): wheelleg profile should not enable ROBOT_TASK_MODULE_ARM."
            }
        }
    }
}

function Test-RtProfilerDescriptors {
    Write-Host "[check] rt profiler descriptors"

    $profilerHeader = Join-Path $script:RepoRoot "shared\application\services\diagnostics\rt_profiler.h"
    $profilerSource = Join-Path $script:RepoRoot "shared\application\services\diagnostics\rt_profiler.c"
    if (-not (Test-Path -LiteralPath $profilerHeader -PathType Leaf)) {
        Add-CheckError "Missing rt profiler header: $(Format-RepoPath $profilerHeader)"
        return
    }
    if (-not (Test-Path -LiteralPath $profilerSource -PathType Leaf)) {
        Add-CheckError "Missing rt profiler source: $(Format-RepoPath $profilerSource)"
        return
    }

    $headerContent = Get-Content -LiteralPath $profilerHeader -Raw
    $sourceContent = Get-Content -LiteralPath $profilerSource -Raw
    $profilerIds = @([regex]::Matches($headerContent, '(?m)^\s*(RT_PROFILER_[A-Z0-9_]+)\s*(?:=|,)') |
        ForEach-Object { $_.Groups[1].Value } |
        Where-Object { $_ -ne "RT_PROFILER_COUNT" } |
        Select-Object -Unique)

    $descMatch = [regex]::Match($sourceContent,
        's_rt_profiler_desc\s*\[[^\]]+\]\s*=\s*\{(?<body>.*?)\};',
        [System.Text.RegularExpressions.RegexOptions]::Singleline)
    if (-not $descMatch.Success) {
        Add-CheckError "$(Format-RepoPath $profilerSource): cannot find s_rt_profiler_desc table."
        return
    }

    $descBody = $descMatch.Groups["body"].Value
    foreach ($profilerId in $profilerIds) {
        if ($descBody -notmatch [regex]::Escape($profilerId)) {
            Add-CheckError "$(Format-RepoPath $profilerSource): $profilerId has no descriptor entry."
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

function Test-SimulationTools {
    Write-Host "[check] simulation tools"

    $simTool = Join-Path $script:RepoRoot "tools\sim\robot_sim.py"
    if (-not (Test-Path -LiteralPath $simTool -PathType Leaf)) {
        Add-CheckWarning "simulation tool not found; skipped pressure simulation smoke checks."
        return
    }

    $python = Get-Command python -ErrorAction SilentlyContinue
    if ($null -eq $python) {
        Add-CheckWarning "python is not available; skipped simulation tool smoke checks."
        return
    }

    foreach ($projectName in @("HERO-C", "MINIWHEELEG-C")) {
        $output = & $python.Source $simTool --project $projectName --json 2>&1
        $jsonText = ($output -join "`n")
        if ($LASTEXITCODE -ne 0) {
            Add-CheckError "tools\sim\robot_sim.py $projectName failed: $jsonText"
            continue
        }

        try {
            $report = $jsonText | ConvertFrom-Json
            if ($report.project.name -ne $projectName) {
                Add-CheckError "tools\sim\robot_sim.py $projectName returned project '$($report.project.name)'."
            }
            if ($null -eq $report.can.buses -or $report.can.buses.Count -eq 0) {
                Add-CheckError "tools\sim\robot_sim.py $projectName returned no CAN bus report."
            }
        }
        catch {
            Add-CheckError "tools\sim\robot_sim.py $projectName returned invalid JSON: $($_.Exception.Message)"
        }
    }
}

function Test-BuildManifestTools {
    param([int]$ExpectedProjectCount)

    Write-Host "[check] build manifest tools"

    $manifestTool = Join-Path $script:RepoRoot "tools\build\project_manifest.py"
    if (-not (Test-Path -LiteralPath $manifestTool -PathType Leaf)) {
        Add-CheckError "Missing build manifest tool: $(Format-RepoPath $manifestTool)"
        return
    }

    $python = Get-Command python -ErrorAction SilentlyContinue
    if ($null -eq $python) {
        Add-CheckWarning "python is not available; skipped build manifest smoke checks."
        return
    }

    $output = & $python.Source $manifestTool --all --check --json --summary-only 2>&1
    $jsonText = ($output -join "`n")
    if ($LASTEXITCODE -ne 0) {
        Add-CheckError "tools\build\project_manifest.py --all --check failed: $jsonText"
        return
    }

    try {
        $report = $jsonText | ConvertFrom-Json
        if ($report.summary.project_count -ne $ExpectedProjectCount) {
            Add-CheckError "tools\build\project_manifest.py returned $($report.summary.project_count) project(s), expected $ExpectedProjectCount."
        }
        if ($report.summary.validation_errors -ne 0) {
            Add-CheckError "tools\build\project_manifest.py reported $($report.summary.validation_errors) validation error(s)."
        }
        if ($null -eq $report.projects -or $report.projects.Count -eq 0) {
            Add-CheckError "tools\build\project_manifest.py returned no project manifests."
        }
    }
    catch {
        Add-CheckError "tools\build\project_manifest.py returned invalid JSON: $($_.Exception.Message)"
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

function Test-HighRateApiBoundaries {
    Write-Host "[check] high-rate API boundaries"

    $highRateFiles = @(
        "shared\application\chassis\chassis_control_task.c",
        "shared\application\gimbal\gimbal_control_task.c",
        "shared\application\shoot\shoot.c",
        "shared\application\comm\can\can_command_tx_task.c",
        "shared\application\wheelleg\wheelleg_mit_task.c"
    )
    $forbiddenPatterns = @(
        [pscustomobject]@{
            Pattern = 'motor_instance_cmd_set_current_many_best_effort\s*\('
            Message = 'resolve motor current outputs during init, then use current bindings in the fast loop.'
        },
        [pscustomobject]@{
            Pattern = 'motor_instance_cmd_set_current_many\s*\('
            Message = 'name-based motor current output is not allowed in high-rate task sources.'
        },
        [pscustomobject]@{
            Pattern = 'motor_instance_cmd_set_current\s*\('
            Message = 'name-based single motor current output is not allowed in high-rate task sources.'
        },
        [pscustomobject]@{
            Pattern = 'motor_instance_find_by_name\s*\('
            Message = 'name lookup belongs in init/config/diagnostics paths, not high-rate task sources.'
        },
        [pscustomobject]@{
            Pattern = 'robot_config_(motor_)?device_find_by_name\s*\('
            Message = 'device-table name lookup belongs in init/config/diagnostics paths, not high-rate task sources.'
        },
        [pscustomobject]@{
            Pattern = 'control_manager_[A-Za-z0-9_]*by_name\s*\('
            Message = 'controller name lookup belongs in command or diagnostics paths, not high-rate task sources.'
        },
        [pscustomobject]@{
            Pattern = 'strcmp\s*\('
            Message = 'string compare is not allowed in high-rate task sources.'
        }
    )

    foreach ($repoPath in $highRateFiles) {
        $fullPath = Join-Path $script:RepoRoot $repoPath
        if (-not (Test-Path -LiteralPath $fullPath -PathType Leaf)) {
            Add-CheckError "Missing high-rate source: $repoPath"
            continue
        }

        $content = Get-Content -LiteralPath $fullPath -Raw
        foreach ($forbidden in $forbiddenPatterns) {
            if ($content -match $forbidden.Pattern) {
                Add-CheckError "${repoPath}: $($forbidden.Message)"
            }
        }
    }
}

function Test-CanTxDeviceConfigBoundaries {
    Write-Host "[check] CAN TX device config boundaries"

    $repoPath = "shared\application\comm\can\can_command_tx_task.c"
    $fullPath = Join-Path $script:RepoRoot $repoPath
    if (-not (Test-Path -LiteralPath $fullPath -PathType Leaf)) {
        Add-CheckError "Missing CAN TX source: $repoPath"
        return
    }

    $content = Get-Content -LiteralPath $fullPath -Raw
    $rmUsesResolvedBus = $content -match 'static\s+inline\s+void\s+can_tx_process_rm_axis[\s\S]*?const\s+uint8_t\s+node_bus\s*=\s*can_tx_node_bus\s*\(\s*fallback_bus\s*,\s*node\s*\)\s*;[\s\S]*?can_tx_store_rm_current\s*\(\s*node_bus\s*,'
    if (-not $rmUsesResolvedBus) {
        Add-CheckError "${repoPath}: RM group send path must use the resolved node CAN bus, not the fixed fallback bus."
    }
}

function Test-ControlRegistryBoundaries {
    Write-Host "[check] control registry boundaries"

    $repoPath = "shared\application\robot\robot_control_registry.h"
    $fullPath = Join-Path $script:RepoRoot $repoPath
    if (-not (Test-Path -LiteralPath $fullPath -PathType Leaf)) {
        Add-CheckError "Missing control registry header: $repoPath"
        return
    }

    $content = Get-Content -LiteralPath $fullPath -Raw
    foreach ($forbidden in @("control_manager_request_switch", "control_manager_update_all", "control_manager_update_due_all")) {
        if ($content -match [regex]::Escape($forbidden)) {
            Add-CheckError "${repoPath}: default controller registry must not mark controllers active during boot via '$forbidden'."
        }
    }
}

function Test-ControlCoreBoundaries {
    Write-Host "[check] control core boundaries"

    $coreFiles = @(
        "shared\application\robot\control_core.h",
        "shared\application\arm\arm_core.h",
        "shared\application\chassis\chassis_core.h",
        "shared\application\gimbal\gimbal_core.h",
        "shared\application\wheelleg\wheelleg_core.h"
    )
    $forbiddenPatterns = @(
        'FreeRTOS\.h',
        'cmsis_os\.h',
        'task\.h',
        'CAN_receive\.h',
        'motor_instance\.h',
        'INS_task\.h',
        'sdlog\.h',
        'watch\.h',
        'config\.h',
        '\bg_config\b',
        '\bosDelay\s*\(',
        '\bxTaskGetTickCount\s*\('
    )

    foreach ($repoPath in $coreFiles) {
        $fullPath = Join-Path $script:RepoRoot $repoPath
        if (-not (Test-Path -LiteralPath $fullPath -PathType Leaf)) {
            Add-CheckError "Missing control core file: $repoPath"
            continue
        }

        $content = Get-Content -LiteralPath $fullPath -Raw
        foreach ($pattern in $forbiddenPatterns) {
            if ($content -match $pattern) {
                Add-CheckError "${repoPath}: core files must stay independent from RTOS/HAL/config runtime dependency '$pattern'."
            }
        }
    }

    $adapterIncludes = @(
        [pscustomobject]@{
            Source = "shared\application\arm\arm_motion.c"
            Include = '#include "arm_core.h"'
            Step = 'arm_core_step_manual'
        },
        [pscustomobject]@{
            Source = "shared\application\chassis\chassis_control_task.c"
            Include = '#include "chassis_core.h"'
            Step = 'chassis_core_step_velocity'
        },
        [pscustomobject]@{
            Source = "shared\application\gimbal\gimbal_control_task.c"
            Include = '#include "gimbal_core.h"'
            Step = 'gimbal_core_step_axis_base'
        },
        [pscustomobject]@{
            Source = "shared\application\wheelleg\wheelleg_mit_task.c"
            Include = '#include "wheelleg_core.h"'
            Step = @(
                'wheelleg_core_calc_kinematics',
                'wheelleg_core_set_wheel_torques',
                'wheelleg_core_lqr_wheel_output',
                'wheelleg_core_target_smooth_update_xy',
                'wheelleg_core_observer_update'
            )
        }
    )

    foreach ($adapter in $adapterIncludes) {
        $fullPath = Join-Path $script:RepoRoot $adapter.Source
        if (-not (Test-Path -LiteralPath $fullPath -PathType Leaf)) {
            Add-CheckError "Missing control adapter source: $($adapter.Source)"
            continue
        }

        $content = Get-Content -LiteralPath $fullPath -Raw
        if ($content -notmatch [regex]::Escape($adapter.Include)) {
            Add-CheckError "$($adapter.Source): must include $($adapter.Include)."
        }
        foreach ($step in @($adapter.Step)) {
            if ($null -ne $step -and $content -notmatch [regex]::Escape($step)) {
                Add-CheckError "$($adapter.Source): must call $step so the firmware path uses the reusable core."
            }
        }
    }
}

function Test-RobotDeviceSchema {
    Write-Host "[check] robot device schema"

    $schemaPath = Join-Path $script:RepoRoot "shared\application\robot\robot_config_schema.h"
    $devicePath = Join-Path $script:RepoRoot "shared\application\robot\robot_device_config.h"
    if (-not (Test-Path -LiteralPath $schemaPath -PathType Leaf)) {
        Add-CheckError "Missing robot config schema header: $(Format-RepoPath $schemaPath)"
        return
    }
    if (-not (Test-Path -LiteralPath $devicePath -PathType Leaf)) {
        Add-CheckError "Missing robot device config header: $(Format-RepoPath $devicePath)"
        return
    }

    $schemaContent = Get-Content -LiteralPath $schemaPath -Raw
    $deviceContent = Get-Content -LiteralPath $devicePath -Raw

    foreach ($macro in @(
            "ROBOT_DEFAULT_DEVICE_TABLE_COUNT",
            "ROBOT_DEVICE_TABLE_KIND_SENSOR",
            "ROBOT_DEVICE_TABLE_KIND_INPUT",
            "ROBOT_DEVICE_TABLE_KIND_COMM",
            "ROBOT_DEVICE_TABLE_KIND_SERVICE",
            "ROBOT_DEVICE_ENTRY_SENSOR",
            "ROBOT_DEVICE_ENTRY_INPUT",
            "ROBOT_DEVICE_ENTRY_COMM",
            "ROBOT_DEVICE_ENTRY_SERVICE"
        )) {
        if ($schemaContent -notmatch "(?m)^\s*#define\s+$macro\b") {
            Add-CheckError "$(Format-RepoPath $schemaPath): missing $macro."
        }
    }

    foreach ($kind in @(
            "ROBOT_CONFIG_DEVICE_KIND_SENSOR",
            "ROBOT_CONFIG_DEVICE_KIND_INPUT",
            "ROBOT_CONFIG_DEVICE_KIND_COMM",
            "ROBOT_CONFIG_DEVICE_KIND_SERVICE"
        )) {
        if ($deviceContent -notmatch "\b$kind\b") {
            Add-CheckError "$(Format-RepoPath $devicePath): missing $kind."
        }
    }

    if ($schemaContent -notmatch '\.count\s*=\s*\(uint8_t\)ROBOT_DEFAULT_DEVICE_TABLE_COUNT') {
        Add-CheckError "$(Format-RepoPath $schemaPath): ROBOT_DEFAULT_DEVICE_TABLE must use ROBOT_DEFAULT_DEVICE_TABLE_COUNT."
    }

    foreach ($deviceName in @(
            "sensor.imu",
            "input.manual",
            "sensor.battery",
            "link.aux_telem",
            "service.sdlog"
        )) {
        if ($schemaContent -notmatch [regex]::Escape($deviceName)) {
            Add-CheckError "$(Format-RepoPath $schemaPath): default device table is missing '$deviceName'."
        }
    }
}

function Test-SharedConfigTypes {
    Write-Host "[check] shared config types"

    $typesPath = Join-Path $script:RepoRoot "shared\application\robot\robot_config_types.h"
    if (-not (Test-Path -LiteralPath $typesPath -PathType Leaf)) {
        Add-CheckError "Missing shared config types header: $(Format-RepoPath $typesPath)"
        return
    }

    $content = Get-Content -LiteralPath $typesPath -Raw
    if ($content -match '\bARBATOS_TARGET_NAME\b|\bROBOT_PROFILE_KIND\s+ROBOT_PROFILE_KIND_|\bROBOT_BOARD_KIND\s+ROBOT_BOARD_KIND_') {
        Add-CheckError "$(Format-RepoPath $typesPath): shared config types must not contain target identity macros."
    }
    if ($content -notmatch 'typedef\s+struct[\s\S]*?\}\s*config_t\s*;') {
        Add-CheckError "$(Format-RepoPath $typesPath): cannot find shared config_t definition."
    }
    if ($content -notmatch '#ifndef\s+MOTOR_ARM_JOINT_COUNT[\s\S]*?#define\s+MOTOR_ARM_JOINT_COUNT\s+6u[\s\S]*?#endif') {
        Add-CheckError "$(Format-RepoPath $typesPath): MOTOR_ARM_JOINT_COUNT must be a target-overridable default."
    }
    if ($content -notmatch '#include\s+"robot_config_schema\.h"') {
        Add-CheckError "$(Format-RepoPath $typesPath): shared config types must include robot_config_schema.h."
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
Test-ProfileIdentity $projects
Test-ProfileProductRules $projects
Test-RtProfilerDescriptors
Test-PythonTools
Test-SimulationTools
Test-BuildManifestTools $projects.Count
Test-StaleText
Test-HighRateApiBoundaries
Test-CanTxDeviceConfigBoundaries
Test-ControlRegistryBoundaries
Test-ControlCoreBoundaries
Test-RobotDeviceSchema
Test-SharedConfigTypes

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
