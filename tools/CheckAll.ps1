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

function Get-SourceContentWithPrivateIncludes {
    param(
        [string]$Path,
        [System.Collections.Generic.HashSet[string]]$Seen = $null
    )

    if ($null -eq $Seen) {
        $Seen = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::OrdinalIgnoreCase)
    }

    $fullPath = [System.IO.Path]::GetFullPath($Path)
    if ($Seen.Contains($fullPath)) {
        Add-CheckError "$(Format-RepoPath $fullPath): recursive private source include."
        return ""
    }

    [void]$Seen.Add($fullPath)
    $content = Get-Content -LiteralPath $fullPath -Raw -Encoding UTF8
    $baseDir = Split-Path -Parent $fullPath
    $includePattern = '(?m)^\s*#\s*include\s+"([A-Za-z0-9_./-]+\.inc)"\s*$'

    $expanded = [regex]::Replace($content, $includePattern, {
            param($match)

            $includePath = [System.IO.Path]::GetFullPath((Join-Path $baseDir $match.Groups[1].Value))
            if (-not (Test-Path -LiteralPath $includePath -PathType Leaf)) {
                Add-CheckError "$(Format-RepoPath $fullPath): missing private source include $($match.Groups[1].Value)."
                return ""
            }

            return Get-SourceContentWithPrivateIncludes -Path $includePath -Seen $Seen
        })

    [void]$Seen.Remove($fullPath)
    return $expanded
}

function Get-RobotConfigContent {
    param(
        [string]$Path,
        [System.Collections.Generic.HashSet[string]]$Seen = $null
    )

    if ($null -eq $Seen) {
        $Seen = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::OrdinalIgnoreCase)
    }

    $fullPath = [System.IO.Path]::GetFullPath($Path)
    if ($Seen.Contains($fullPath)) {
        Add-CheckError "$(Format-RepoPath $fullPath): recursive config include."
        return ""
    }

    [void]$Seen.Add($fullPath)
    $content = Get-Content -LiteralPath $fullPath -Raw -Encoding UTF8
    $baseDir = Split-Path -Parent $fullPath
    $includePattern = '(?m)^\s*#\s*include\s+"(Config[A-Za-z0-9_]+\.inc)"\s*$'

    $expanded = [regex]::Replace($content, $includePattern, {
            param($match)

            $includePath = Join-Path $baseDir $match.Groups[1].Value
            if (-not (Test-Path -LiteralPath $includePath -PathType Leaf)) {
                Add-CheckError "$(Format-RepoPath $fullPath): missing config include $($match.Groups[1].Value)."
                return ""
            }

            return Get-RobotConfigContent -Path $includePath -Seen $Seen
        })

    [void]$Seen.Remove($fullPath)
    return $expanded
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

function Get-NamedTaskModuleMap {
    param([string]$Content)

    $map = @{}
    foreach ($match in [regex]::Matches($Content, '\{\s*(ROBOT_TASK_MODULE_[A-Z0-9_]+)\s*,\s*"(task\.[^"]+)"\s*\}')) {
        $map[$match.Groups[1].Value] = $match.Groups[2].Value
    }

    return $map
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
    $configC = Join-Path $configDir "RobotConfig.c"
    $configH = Join-Path $configDir "RobotConfig.h"

    if (-not (Test-Path -LiteralPath $configC -PathType Leaf)) {
        Add-CheckError "Missing Robotconfig RobotConfig.c for $($Project.Name): $(Format-RepoPath $configC)"
        return
    }

    if (-not (Test-Path -LiteralPath $configH -PathType Leaf)) {
        Add-CheckError "Missing Robotconfig RobotConfig.h for $($Project.Name): $(Format-RepoPath $configH)"
        return
    }

    $configContent = Get-RobotConfigContent -Path $configC
    $configHeader = Get-Content -LiteralPath $configH -Raw
    $moduleCount = Get-ProfileModuleCount $configContent
    $modules = @(Get-ProfileModules $configContent)

    if ($configHeader -notmatch '#include\s+"RobotConfigTypes\.h"') {
        Add-CheckError "$(Format-RepoPath $configH): target RobotConfig.h must include the shared RobotConfigTypes.h."
    }
    if ($configHeader -match '(?m)^\s*typedef\s+(struct|enum)\b') {
        Add-CheckError "$(Format-RepoPath $configH): target RobotConfig.h must only carry target identity/build macros; common config types belong in shared\application\robot\RobotConfigTypes.h."
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
    if ($configHeader -match 'RobotConfigDeviceTable\s+devices\s*;' -and
        $configContent -notmatch '\.devices\s*=') {
        Add-CheckError "$(Format-RepoPath $configC): cannot find .devices initializer; MotorInst now uses the runtime device table."
    }
    if ($configContent -match '\.devices\s*=\s*\{\s*\.count\s*=\s*0u?\s*[,}]') {
        Add-CheckError "$(Format-RepoPath $configC): runtime device table count is 0; MotorInst would have no configured devices."
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

    if ($taskText -notmatch 'AppCreateEnabledModuleTasks') {
        Add-CheckError "$(Format-RepoPath $Project.UvprojxPath): task creation must use shared AppCreateEnabledModuleTasks()."
    }
    if ($taskText -notmatch 'RobotControlBootstrapProfileDefaults') {
        Add-CheckError "$(Format-RepoPath $Project.UvprojxPath): FreeRTOS init must bootstrap default controllers with RobotControlBootstrapProfileDefaults()."
    }
    if ($taskText -match 'typedef\s+struct[\s\S]*?AppTaskModuleDesc') {
        Add-CheckError "$(Format-RepoPath $Project.UvprojxPath): task source must use AppTaskBootstrap.h instead of redefining AppTaskModuleDesc locally."
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
        Test-RequiredSource $Project $SourceSet "shared\application\chassis\ChassisControlTask.c"
        Test-RequiredTaskText $Project $TaskText "ROBOT_TASK_MODULE_CLASSIC_CHASSIS"
        Test-RequiredTaskText $Project $TaskText "ChassisControlTask"
    }
    if (Test-ProfileHasModule $Modules "ROBOT_TASK_MODULE_WHEELLEG_MIT") {
        Test-RequiredSource $Project $SourceSet "shared\application\wheelleg\WheelLegMitTask.c"
        Test-RequiredTaskText $Project $TaskText "ROBOT_TASK_MODULE_WHEELLEG_MIT"
        Test-RequiredTaskText $Project $TaskText "WheelLegMitTask"
    }
    if (Test-ProfileHasModule $Modules "ROBOT_TASK_MODULE_WHEELLEG_SERVO") {
        Add-CheckError "$(Format-RepoPath $Project.UvprojxPath): profile lists WHEELLEG_SERVO, but no servo wheel-leg task is wired yet."
    }
    if (Test-ProfileHasModule $Modules "ROBOT_TASK_MODULE_SINGLE_GIMBAL") {
        Test-RequiredSource $Project $SourceSet "shared\application\gimbal\GimbalControlTask.c"
        Test-RequiredTaskText $Project $TaskText "ROBOT_TASK_MODULE_SINGLE_GIMBAL"
        Test-RequiredTaskText $Project $TaskText "GimbalControlTask"
    }
    if (Test-ProfileHasModule $Modules "ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL") {
        Test-RequiredSource $Project $SourceSet "shared\application\gimbal\GimbalControlTask.c"
        Test-RequiredTaskText $Project $TaskText "ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL"
        Test-RequiredTaskText $Project $TaskText "DualYawGimbalControlTask"
    }
    if (Test-ProfileHasModule $Modules "ROBOT_TASK_MODULE_ARM") {
        Test-RequiredSource $Project $SourceSet "shared\application\arm\ArmTask.c"
        Test-RequiredTaskText $Project $TaskText "ROBOT_TASK_MODULE_ARM"
        Test-RequiredTaskText $Project $TaskText "ArmTask"
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

function Expand-ConfigParamRange {
    param([string]$RangeText)

    $ids = New-Object System.Collections.Generic.List[int]
    if ([string]::IsNullOrWhiteSpace($RangeText)) {
        return $ids
    }

    foreach ($part in $RangeText.Split(",")) {
        $trimmed = $part.Trim()
        if ($trimmed -eq "") {
            continue
        }

        $range = [regex]::Match($trimmed, '^(\d+)\s*-\s*(\d+)$')
        if ($range.Success) {
            $start = [int]$range.Groups[1].Value
            $end = [int]$range.Groups[2].Value
            if ($start -gt $end) {
                $tmp = $start
                $start = $end
                $end = $tmp
            }
            for ($id = $start; $id -le $end; $id++) {
                $ids.Add($id)
            }
            continue
        }

        $single = [regex]::Match($trimmed, '^(\d+)$')
        if ($single.Success) {
            $ids.Add([int]$single.Groups[1].Value)
            continue
        }

        Add-CheckError "Bad config param range item '$trimmed'."
    }

    return $ids
}

function Test-ConfigParamGovernance {
    Write-Host "[check] config param governance"

    $paramPath = Join-Path $script:RepoRoot "shared\application\robot\ConfigParamList.inc"
    if (-not (Test-Path -LiteralPath $paramPath -PathType Leaf)) {
        Add-CheckError "Missing config parameter list: $(Format-RepoPath $paramPath)"
        return
    }

    $content = Get-Content -LiteralPath $paramPath -Raw -Encoding UTF8
    $pattern = '(?m)^\s*CONFIG_PARAM_(F32_RANGE|F32|U16|U8|I8_RANGE|BOOL|U8_MAX|U8_DEFAULT)\(\s*(\d+)u?\s*,\s*"([^"]+)"\s*,\s*([A-Z0-9_]+)\s*,\s*(g_config\.[^,\)]+)'
    $matches = [regex]::Matches($content, $pattern)
    if ($matches.Count -eq 0) {
        Add-CheckError "$(Format-RepoPath $paramPath): no CONFIG_PARAM entries found."
        return
    }

    $ids = @{}
    $names = @{}
    $paramIds = [System.Collections.Generic.HashSet[int]]::new()
    $paramNames = @{}

    foreach ($match in $matches) {
        $id = [int]$match.Groups[2].Value
        $name = $match.Groups[3].Value
        $field = $match.Groups[5].Value.Trim()

        [void]$paramIds.Add($id)
        $paramNames[$id] = $name

        if (-not $ids.ContainsKey($id)) {
            $ids[$id] = New-Object System.Collections.Generic.List[string]
        }
        $ids[$id].Add($name)

        if (-not $names.ContainsKey($name)) {
            $names[$name] = New-Object System.Collections.Generic.List[int]
        }
        $names[$name].Add($id)

        if ($field -notmatch '^g_config\.') {
            Add-CheckError "$(Format-RepoPath $paramPath): param $id '$name' does not write into g_config."
        }
    }

    foreach ($entry in $ids.GetEnumerator()) {
        if ($entry.Value.Count -gt 1) {
            Add-CheckError "$(Format-RepoPath $paramPath): duplicate config param id $($entry.Key): $($entry.Value -join ', ')."
        }
    }

    foreach ($entry in $names.GetEnumerator()) {
        if ($entry.Value.Count -gt 1) {
            Add-CheckError "$(Format-RepoPath $paramPath): duplicate config param name '$($entry.Key)': $($entry.Value -join ', ')."
        }
    }

    $robotConfigFiles = @(Get-ChildItem -Path (Join-Path $script:RepoRoot "Robotconfig") -Filter "RobotConfig.c" -Recurse | Sort-Object FullName)
    foreach ($file in $robotConfigFiles) {
        $configContent = Get-Content -LiteralPath $file.FullName -Raw -Encoding UTF8
        if ($configContent -match 'g_config_active_blocks') {
            Add-CheckError "$(Format-RepoPath $file.FullName): config_get_block_table must return the full block table; use config_block_is_active for enable state."
        }

        $covered = [System.Collections.Generic.HashSet[int]]::new()
        $blockMatches = [regex]::Matches($configContent, '\{CONFIG_BLOCK_[^,]+,\s*"[^"]+"\s*,\s*"([^"]*)"')
        foreach ($blockMatch in $blockMatches) {
            foreach ($id in (Expand-ConfigParamRange $blockMatch.Groups[1].Value)) {
                [void]$covered.Add($id)
            }
        }

        foreach ($id in $paramIds) {
            if (-not $covered.Contains($id)) {
                Add-CheckError "$(Format-RepoPath $file.FullName): config param $id '$($paramNames[$id])' is not covered by any block param_range."
            }
        }
    }
}

function Test-TaskModuleNames {
    param([object[]]$Projects)

    Write-Host "[check] task module names"

    $profileHeader = Join-Path $script:RepoRoot "shared\application\robot\RobotTaskProfile.h"
    $schemaHeader = Join-Path $script:RepoRoot "shared\application\robot\RobotConfigSchema.h"
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

function Test-RobotModuleDescriptors {
    Write-Host "[check] robot module descriptors"

    $moduleHeader = Join-Path $script:RepoRoot "shared\application\robot\RobotModule.h"
    $profileHeader = Join-Path $script:RepoRoot "shared\application\robot\RobotTaskProfile.h"
    $schemaHeader = Join-Path $script:RepoRoot "shared\application\robot\RobotConfigSchema.h"
    foreach ($path in @($moduleHeader, $profileHeader, $schemaHeader)) {
        if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
            Add-CheckError "Missing robot module check input: $(Format-RepoPath $path)"
            return
        }
    }

    $moduleContent = Get-Content -LiteralPath $moduleHeader -Raw -Encoding UTF8
    $profileContent = Get-Content -LiteralPath $profileHeader -Raw -Encoding UTF8
    $schemaContent = Get-Content -LiteralPath $schemaHeader -Raw -Encoding UTF8
    $profileNames = Get-NamedTaskModuleMap $profileContent

    $descMatch = [regex]::Match($moduleContent,
        'RobotModuleDesc\s+modules\[\]\s*=\s*\{(?<body>.*?)\};',
        [System.Text.RegularExpressions.RegexOptions]::Singleline)
    if (-not $descMatch.Success) {
        Add-CheckError "$(Format-RepoPath $moduleHeader): cannot find RobotModuleDesc modules[] table."
        return
    }

    $descByModule = @{}
    $moduleNames = @{}
    $descPattern = '\{\s*(ROBOT_TASK_MODULE_[A-Z0-9_]+)\s*,\s*"(module\.[^"]+)"\s*,\s*"(task\.[^"]+)"'
    foreach ($match in [regex]::Matches($descMatch.Groups["body"].Value, $descPattern)) {
        $module = $match.Groups[1].Value
        $moduleName = $match.Groups[2].Value
        $taskName = $match.Groups[3].Value

        if ($descByModule.ContainsKey($module)) {
            Add-CheckError "$(Format-RepoPath $moduleHeader): duplicate descriptor for $module."
        }
        $descByModule[$module] = [pscustomobject]@{
            ModuleName = $moduleName
            TaskName = $taskName
        }

        if ($moduleName -notmatch '^module\.[a-z0-9_]+$') {
            Add-CheckError "$(Format-RepoPath $moduleHeader): $module uses non-standard module name '$moduleName'."
        }
        if ($moduleNames.ContainsKey($moduleName)) {
            Add-CheckError "$(Format-RepoPath $moduleHeader): duplicate module name '$moduleName'."
        }
        $moduleNames[$moduleName] = $module

        if (-not $profileNames.ContainsKey($module)) {
            Add-CheckError "$(Format-RepoPath $moduleHeader): $module has descriptor task name '$taskName', but no task name in RobotTaskProfile.h."
        }
        elseif ($profileNames[$module] -ne $taskName) {
            Add-CheckError "$(Format-RepoPath $moduleHeader): $module descriptor task name '$taskName' differs from RobotTaskProfile.h '$($profileNames[$module])'."
        }
    }

    foreach ($module in Get-TaskModuleEnums $schemaContent) {
        if (-not $descByModule.ContainsKey($module)) {
            Add-CheckError "$(Format-RepoPath $moduleHeader): $module has no RobotModuleDesc entry."
        }
    }

    $resourceEnumMatch = [regex]::Match($moduleContent,
        'typedef\s+enum\s*\{(?<body>.*?)\}\s*RobotResourceId\s*;',
        [System.Text.RegularExpressions.RegexOptions]::Singleline)
    if (-not $resourceEnumMatch.Success) {
        Add-CheckError "$(Format-RepoPath $moduleHeader): cannot find RobotResourceId enum."
        return
    }

    $resourceEnums = New-Object System.Collections.Generic.HashSet[string]
    foreach ($match in [regex]::Matches($resourceEnumMatch.Groups["body"].Value, 'RobotResource[A-Za-z0-9_]+')) {
        if ($match.Value -ne "RobotResourceCount") {
            [void]$resourceEnums.Add($match.Value)
        }
    }

    $resourceTableMatch = [regex]::Match($moduleContent,
        'RobotResourceDesc\s+resources\[\]\s*=\s*\{(?<body>.*?)\};',
        [System.Text.RegularExpressions.RegexOptions]::Singleline)
    if (-not $resourceTableMatch.Success) {
        Add-CheckError "$(Format-RepoPath $moduleHeader): cannot find RobotResourceDesc resources[] table."
        return
    }

    $resourceNames = @{}
    $resourceDesc = New-Object System.Collections.Generic.HashSet[string]
    foreach ($match in [regex]::Matches($resourceTableMatch.Groups["body"].Value, '\{\s*(RobotResource[A-Za-z0-9_]+)\s*,\s*"([^"]+)"\s*\}')) {
        $resource = $match.Groups[1].Value
        $name = $match.Groups[2].Value
        [void]$resourceDesc.Add($resource)

        if (-not $resourceEnums.Contains($resource)) {
            Add-CheckError "$(Format-RepoPath $moduleHeader): resource table names unknown $resource."
        }
        if ($name -notmatch '^(resource|input|runtime|actuator|bus|port|link|sensor|device|state|output|service)\.[a-z0-9_]+$') {
            Add-CheckError "$(Format-RepoPath $moduleHeader): $resource uses non-standard resource name '$name'."
        }
        if ($resourceNames.ContainsKey($name)) {
            Add-CheckError "$(Format-RepoPath $moduleHeader): duplicate resource name '$name'."
        }
        $resourceNames[$name] = $resource
    }

    foreach ($resource in $resourceEnums) {
        if (-not $resourceDesc.Contains($resource)) {
            Add-CheckError "$(Format-RepoPath $moduleHeader): $resource has no RobotResourceDesc entry."
        }
    }

    $resourceArrayPattern = 'sRobotModule(?:Req|Pro)[A-Za-z0-9_]+\[\]\s*=\s*\{(?<body>.*?)\};'
    foreach ($arrayMatch in [regex]::Matches($moduleContent, $resourceArrayPattern, [System.Text.RegularExpressions.RegexOptions]::Singleline)) {
        foreach ($resourceMatch in [regex]::Matches($arrayMatch.Groups["body"].Value, 'RobotResource[A-Za-z0-9_]+')) {
            if (-not $resourceEnums.Contains($resourceMatch.Value)) {
                Add-CheckError "$(Format-RepoPath $moduleHeader): module resource array uses unknown $($resourceMatch.Value)."
            }
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
        $configHeader = Join-Path $script:RepoRoot "Robotconfig\$($project.Name)\RobotConfig.h"
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
        "SENTINEL-M" = "ROBOT_PROFILE_KIND_SENTRY"
    }

    foreach ($project in $Projects) {
        $configHeader = Join-Path $script:RepoRoot "Robotconfig\$($project.Name)\RobotConfig.h"
        $configC = Join-Path $script:RepoRoot "Robotconfig\$($project.Name)\RobotConfig.c"
        if (-not (Test-Path -LiteralPath $configHeader -PathType Leaf) -or
            -not (Test-Path -LiteralPath $configC -PathType Leaf)) {
            continue
        }

        $headerContent = Get-Content -LiteralPath $configHeader -Raw
        $configContent = Get-RobotConfigContent -Path $configC
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
                    '\.WheelLegMit\s*=\s*\{\s*0\s*\}',
                    '\.ArmJ0Unitree\s*=\s*\{\s*0\s*\}'
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

    $profilerHeader = Join-Path $script:RepoRoot "shared\application\services\diagnostics\RtProf.h"
    $profilerSource = Join-Path $script:RepoRoot "shared\application\services\diagnostics\RtProf.c"
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
        Where-Object { $_ -ne "RtProfCount" } |
        Select-Object -Unique)

    $descMatch = [regex]::Match($sourceContent,
        'sRtProfDesc\s*\[[^\]]+\]\s*=\s*\{(?<body>.*?)\};',
        [System.Text.RegularExpressions.RegexOptions]::Singleline)
    if (-not $descMatch.Success) {
        Add-CheckError "$(Format-RepoPath $profilerSource): cannot find sRtProfDesc table."
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

    $simTool = Join-Path $script:RepoRoot "tools\sim\RobotSim.py"
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
            Add-CheckError "tools\sim\RobotSim.py $projectName failed: $jsonText"
            continue
        }

        try {
            $report = $jsonText | ConvertFrom-Json
            if ($report.project.name -ne $projectName) {
                Add-CheckError "tools\sim\RobotSim.py $projectName returned project '$($report.project.name)'."
            }
            if ($null -eq $report.can.buses -or $report.can.buses.Count -eq 0) {
                Add-CheckError "tools\sim\RobotSim.py $projectName returned no CAN bus report."
            }
        }
        catch {
            Add-CheckError "tools\sim\RobotSim.py $projectName returned invalid JSON: $($_.Exception.Message)"
        }
    }

    $mujocoWheellegTool = Join-Path $script:RepoRoot "tools\mujoco\wheelleg\RunWheelLeg.py"
    if (Test-Path -LiteralPath $mujocoWheellegTool -PathType Leaf) {
        $output = & $python.Source $mujocoWheellegTool --check --project MINIWHEELEG-C 2>&1
        if ($LASTEXITCODE -ne 0) {
            Add-CheckError "tools\mujoco\wheelleg\RunWheelLeg.py --check failed: $($output -join "`n")"
        }
    }
}

function Test-BuildManifestTools {
    param([int]$ExpectedProjectCount)

    Write-Host "[check] build manifest tools"

    $manifestTool = Join-Path $script:RepoRoot "tools\build\ProjectManifest.py"
    $gccTool = Join-Path $script:RepoRoot "tools\build\GccProject.py"
    if (-not (Test-Path -LiteralPath $manifestTool -PathType Leaf)) {
        Add-CheckError "Missing build manifest tool: $(Format-RepoPath $manifestTool)"
        return
    }
    if (-not (Test-Path -LiteralPath $gccTool -PathType Leaf)) {
        Add-CheckError "Missing GCC build generator: $(Format-RepoPath $gccTool)"
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
        Add-CheckError "tools\build\ProjectManifest.py --all --check failed: $jsonText"
        return
    }

    try {
        $report = $jsonText | ConvertFrom-Json
        if ($report.summary.project_count -ne $ExpectedProjectCount) {
            Add-CheckError "tools\build\ProjectManifest.py returned $($report.summary.project_count) project(s), expected $ExpectedProjectCount."
        }
        if ($report.summary.validation_errors -ne 0) {
            Add-CheckError "tools\build\ProjectManifest.py reported $($report.summary.validation_errors) validation error(s)."
        }
        if ($null -eq $report.projects -or $report.projects.Count -eq 0) {
            Add-CheckError "tools\build\ProjectManifest.py returned no project manifests."
        }
    }
    catch {
        Add-CheckError "tools\build\ProjectManifest.py returned invalid JSON: $($_.Exception.Message)"
    }

    $gccOutput = & $python.Source $gccTool --all --check-only --json 2>&1
    $gccJsonText = ($gccOutput -join "`n")
    if ($LASTEXITCODE -ne 0) {
        Add-CheckError "tools\build\GccProject.py --all --check-only failed: $gccJsonText"
        return
    }

    try {
        $gccReport = $gccJsonText | ConvertFrom-Json
        foreach ($result in $gccReport.results) {
            if ($result.blockers.Count -ne 0) {
                Add-CheckError "GCC generator blocked for $($result.project): $($result.blockers -join '; ')"
            }
        }
    }
    catch {
        Add-CheckError "tools\build\GccProject.py returned invalid JSON: $($_.Exception.Message)"
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
            Pattern = 'MIT wheel-leg.*no actual|wheel-leg MIT.*not wired|WheelLegMitTask.*not wired'
            Message = 'MIT wheel-leg task is already wired; this sentence is stale.'
        },
        [pscustomobject]@{
            Pattern = 'WheelLegMitTask.*missing'
            Message = 'WheelLegMitTask already exists and is wired in current entries.'
        }
    )

    foreach ($repoPath in Get-TextFilesToCheck) {
        if ($repoPath.Replace("/", "\") -eq "tools\CheckAll.ps1") {
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

function Test-IncludeFilenameCase {
    Write-Host "[check] include filename case"

    $fileMap = [System.Collections.Generic.Dictionary[string, System.Collections.Generic.List[object]]]::new(
        [System.StringComparer]::OrdinalIgnoreCase)

    foreach ($path in (git -C $script:RepoRoot -c core.quotepath=false ls-files)) {
        $name = ($path -split '/|\\')[-1]
        if (-not $fileMap.ContainsKey($name)) {
            $fileMap[$name] = [System.Collections.Generic.List[object]]::new()
        }
        $fileMap[$name].Add([pscustomobject]@{
                Name = $name
                Path = $path
            })
    }

    $sourceExt = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::OrdinalIgnoreCase)
    foreach ($ext in @(".c", ".h", ".inc", ".cpp", ".hpp", ".s", ".S")) {
        [void]$sourceExt.Add($ext)
    }

    foreach ($path in (git -C $script:RepoRoot -c core.quotepath=false ls-files)) {
        $normPath = $path -replace '\\', '/'
        if ($normPath -match '^projects/[^/]+/(Drivers|Middlewares)/') {
            continue
        }
        if ($normPath -match '^tools/build/gcc_support/') {
            continue
        }
        if ($normPath -match '^shared/components/algorithm/Include/') {
            continue
        }
        if ($normPath -match '^shared/components/support/fatfs/') {
            continue
        }

        $name = ($normPath -split '/')[-1]
        $dot = $name.LastIndexOf(".")
        if ($dot -lt 0 -or -not $sourceExt.Contains($name.Substring($dot))) {
            continue
        }

        $fullPath = Join-Path $script:RepoRoot $normPath
        if (-not (Test-Path -LiteralPath $fullPath -PathType Leaf)) {
            continue
        }

        try {
            $content = Get-Content -LiteralPath $fullPath -Raw -Encoding UTF8
        }
        catch {
            continue
        }

        foreach ($match in [regex]::Matches($content, '(?m)^\s*#\s*include\s+"([^"]+)"')) {
            $include = $match.Groups[1].Value
            $includeName = ($include -split '/|\\')[-1]
            if (-not $fileMap.ContainsKey($includeName)) {
                continue
            }

            $hasExactCase = $false
            foreach ($candidate in $fileMap[$includeName]) {
                if ($candidate.Name -ceq $includeName) {
                    $hasExactCase = $true
                    break
                }
            }

            if (-not $hasExactCase) {
                $actualNames = ($fileMap[$includeName] | ForEach-Object { $_.Name } | Select-Object -Unique) -join ", "
                Add-CheckError "$(Format-RepoPath $fullPath): include '$include' differs from tracked file name: $actualNames."
            }
        }
    }
}

function Test-ProjectOwnedPathNames {
    Write-Host "[check] project-owned path names"

    $allowedTargets = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::Ordinal)
    foreach ($target in @("CARRIER-A", "HERO-C", "HERO-M", "INFANTRY-A", "MINIWHEELEG-C", "MINIWHEELEG-M", "SENTINEL-M")) {
        [void]$allowedTargets.Add($target)
    }

    $reported = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::OrdinalIgnoreCase)

    foreach ($path in (git -C $script:RepoRoot -c core.quotepath=false ls-files)) {
        $normPath = $path -replace '\\', '/'
        if ($normPath -match '^projects/[^/]+/(Drivers|Middlewares|Core|USB_DEVICE)/') {
            continue
        }
        if ($normPath -match '^tools/build/gcc_support/') {
            continue
        }
        if ($normPath -match '^tools/tests/(?:[^/]+-)?stubs/cmsis_compiler\.h$') {
            continue
        }
        if ($normPath -match '^shared/components/algorithm/Include/') {
            continue
        }
        if ($normPath -match '^shared/components/support/fatfs/') {
            continue
        }
        if ($normPath -match '^\.github/PULL_REQUEST_TEMPLATE\.md$') {
            continue
        }
        if ($normPath -match '^projects/[^/]+/MDK-ARM/startup_stm32[a-z0-9]+\.s$') {
            continue
        }
        if ($normPath -match '^projects/[^/]+/MDK-ARM/[^/]+_ccm\.sct$') {
            continue
        }
        if ($normPath -eq 'shared/components/algorithm/arm_cortexM4lf_math.lib') {
            continue
        }
        if ($normPath -match '^tools/Mp3ToU8/(FFMPEG_LICENSE\.txt|FFMPEG_README\.txt|ffmpeg\.exe|\.gitignore)$') {
            continue
        }
        if ($normPath -match '^tools/Mp3ToU8/U8/') {
            continue
        }

        $parts = $normPath -split '/'
        for ($i = 0; $i -lt $parts.Count; $i++) {
            $name = $parts[$i]
            if ($name -eq "LICENSE") {
                continue
            }
            if ($i -eq 1 -and ($parts[0] -eq "projects" -or $parts[0] -eq "Robotconfig") -and $allowedTargets.Contains($name)) {
                continue
            }
            if ($i -eq 2 -and $parts[0] -eq "projects" -and $name -eq "MDK-ARM") {
                continue
            }

            if ($name -notmatch '_' -and $name -cnotmatch '^[A-Z0-9_.-]+$') {
                continue
            }

            $componentPath = ($parts[0..$i] -join '/')
            if (-not $reported.Add($componentPath)) {
                continue
            }
            Add-CheckError "$(Format-RepoPath (Join-Path $script:RepoRoot $componentPath)): project-owned path name should avoid underscores or all-uppercase words unless it is generated, vendor-owned, or a fixed external identity."
        }
    }
}

function Test-HighRateApiBoundaries {
    Write-Host "[check] high-rate API boundaries"

    $highRateFiles = @(
        "shared\application\chassis\ChassisControlTask.c",
        "shared\application\gimbal\GimbalControlTask.c",
        "shared\application\shoot\Shoot.c",
        "shared\application\shoot\ShootCtrl.c",
        "shared\application\comm\can\CanTxTask.c",
        "shared\application\wheelleg\WheelLegMitTask.c"
    )
    $forbiddenPatterns = @(
        [pscustomobject]@{
            Pattern = 'MotorInstSetCurrentManyBestEffort\s*\('
            Message = 'resolve motor current outputs during init, then use current bindings in the fast loop.'
        },
        [pscustomobject]@{
            Pattern = 'MotorInstSetCurrentMany\s*\('
            Message = 'name-based motor current output is not allowed in high-rate task sources.'
        },
        [pscustomobject]@{
            Pattern = 'MotorInstSetCurrent\s*\('
            Message = 'name-based single motor current output is not allowed in high-rate task sources.'
        },
        [pscustomobject]@{
            Pattern = 'MotorInstFindByName\s*\('
            Message = 'name lookup belongs in init/config/diagnostics paths, not high-rate task sources.'
        },
        [pscustomobject]@{
            Pattern = 'RobotConfig(Motor)?DeviceFindByName\s*\('
            Message = 'device-table name lookup belongs in init/config/diagnostics paths, not high-rate task sources.'
        },
        [pscustomobject]@{
            Pattern = 'ControlMgr[A-Za-z0-9]*ByName\s*\('
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

        $content = Get-SourceContentWithPrivateIncludes -Path $fullPath
        foreach ($forbidden in $forbiddenPatterns) {
            if ($content -match $forbidden.Pattern) {
                Add-CheckError "${repoPath}: $($forbidden.Message)"
            }
        }
    }
}

function Test-CanRxAndStackSamplingBoundaries {
    Write-Host "[check] CAN RX fairness and stack sampling boundaries"

    $canRepoPath = "shared\hal\BspCan.c"
    $canPath = Join-Path $script:RepoRoot $canRepoPath
    if (-not (Test-Path -LiteralPath $canPath -PathType Leaf)) {
        Add-CheckError "Missing CAN BSP source: $canRepoPath"
    }
    else {
        $canContent = Get-Content -LiteralPath $canPath -Raw
        foreach ($required in @(
                "BSP_CAN_BUS_COUNT",
                "can_rx_next_bus",
                "BspCanRxTryPopBus",
                "checked < BSP_CAN_BUS_COUNT"
            )) {
            if ($canContent -notmatch [regex]::Escape($required)) {
                Add-CheckError "${canRepoPath}: CAN RX dequeue must rotate fairly between enabled buses using '$required'."
            }
        }
        if ($canContent -notmatch 'BspCanRxTryPopBus\s*\(\s*bus\s*,\s*out\s*\)[\s\S]{0,300}?can_rx_next_bus\s*=') {
            Add-CheckError "${canRepoPath}: advance the CAN RX start bus only after a frame is dequeued."
        }
    }

    $stackSources = @(
        [pscustomobject]@{
            Path = "shared\application\gimbal\GimbalControlTask.c"
            Period = "GIMBAL_STACK_SAMPLE_PERIOD_MS"
            Helper = "GimbalStackSampleMaybe"
        },
        [pscustomobject]@{
            Path = "shared\application\chassis\ChassisControlTask.c"
            Period = "CHASSIS_STACK_SAMPLE_PERIOD_MS"
            Helper = "ChassisStackSampleMaybe"
        }
    )

    foreach ($source in $stackSources) {
        $fullPath = Join-Path $script:RepoRoot $source.Path
        if (-not (Test-Path -LiteralPath $fullPath -PathType Leaf)) {
            Add-CheckError "Missing control source: $($source.Path)"
            continue
        }

        $content = Get-Content -LiteralPath $fullPath -Raw
        $scanCount = [regex]::Matches($content, 'uxTaskGetStackHighWaterMark\s*\(').Count
        if ($scanCount -ne 1) {
            Add-CheckError "$($source.Path): stack watermark scanning must exist only in its rate-limited helper."
        }
        if ($content -notmatch "#define\s+$($source.Period)\s+1000u" -or
            $content -notmatch "static\s+void\s+$($source.Helper)\s*\(" -or
            $content -notmatch "\(now\s*-\s*last_sample_tick\)\s*<\s*period") {
            Add-CheckError "$($source.Path): stack watermark sampling must remain rate-limited to about 1 Hz."
        }
    }
}

function Test-CanTxDeviceConfigBoundaries {
    Write-Host "[check] CAN TX device config boundaries"

    $repoPath = "shared\application\comm\can\CanTxTask.c"
    $n6014bRepoPath = "shared\application\motors\N6014bMotorDriver.c"
    $unitreeRepoPath = "shared\application\motors\UnitreeMotorDriver.c"
    $rs485BspRepoPath = "boards\DmMc02H7\bsp\BspUsart.c"
    $fullPath = Join-Path $script:RepoRoot $repoPath
    if (-not (Test-Path -LiteralPath $fullPath -PathType Leaf)) {
        Add-CheckError "Missing CAN TX source: $repoPath"
        return
    }

    $content = Get-SourceContentWithPrivateIncludes -Path $fullPath
    $rmUsesResolvedBus = $content -match 'static\s+inline\s+void\s+CanTxProcessRmAxis[\s\S]*?const\s+uint8_t\s+node_bus\s*=\s*CanTxNodeBus\s*\(\s*fallback_bus\s*,\s*node\s*\)\s*;[\s\S]*?CanTxStoreRmCurrent\s*\(\s*node_bus\s*,'
    if (-not $rmUsesResolvedBus) {
        Add-CheckError "${repoPath}: RM group send path must use the resolved node CAN bus, not the fixed fallback bus."
    }

    if ($content -match '\bDBUS_TOE\b|toe_is_error\s*\(|RobotSafetyManual(?:Disconnected|SafeActive)\s*\(|RobotLifecycle(?!Update\b)\w*\s*\(|ManualInput\w*\s*\(|ControlInput\w*\s*\(') {
        Add-CheckError "${repoPath}: transport must not own source-specific manual-input fault policy."
    }
    $canTxTaskBody = [regex]::Match(
        $content,
        'void\s+CanTxTask\s*\([^;]*\)\s*\{[\s\S]*$'
    ).Value
    if ([string]::IsNullOrWhiteSpace($canTxTaskBody) -or
        ([regex]::Matches($canTxTaskBody, 'RobotLifecycleUpdate\s*\(')).Count -ne 1 -or
        $canTxTaskBody -notmatch 'CanTxOutputGateSync\s*\(\s*RobotSafetyOutputLocked\s*\(\s*\)\s*\)\s*;[\s\S]{0,160}?RobotLifecycleUpdate\s*\(\s*\)\s*;[\s\S]{0,200}?output_locked\s*=\s*RobotSafetyOutputLocked[\s\S]{0,200}?CanTxOutputGateSync\s*\(\s*output_locked\s*\)[\s\S]{0,120}?CanTxExecInstances') {
        Add-CheckError "${repoPath}: CanTx must observe the previous safety state before its sole Lifecycle update, then gate the current frame."
    }
    if ($content -match 'CanTxRouteAllowedOffline|CanTxRouteAllowedOnline|CanTxExecInstances\s*\(\s*uint8_t\s+online' -or
        $content -notmatch 'CanTxExecInstances\s*\(\s*uint8_t\s+output_locked\s*\)') {
        Add-CheckError "${repoPath}: CAN TX may gate by output lock and per-axis command policy, not by an online/offline role table."
    }
    if ($content -notmatch 'const\s+uint8_t\s+output_locked\s*=\s*RobotSafetyOutputLocked\s*\(\s*\)\s*;' -or
        $content -notmatch 'static\s+uint8_t\s+CanTxRouteAllowed\s*\(\s*const\s+MotorRoute\s*\*\s*route\s*\)' -or
        $content -notmatch 'allowed\s*=\s*\(\s*output_locked\s*!=\s*0u\s*\|\|\s*RobotSafetyOutputLocked\s*\(\s*\)\s*!=\s*0u\s*\)\s*\?') {
        Add-CheckError "${repoPath}: the frame lock and a fresh Lifecycle read must flow into every route decision."
    }
    if ($content -notmatch 'static\s+uint8_t\s+CanTxRecheckCommandAuthority\s*\([\s\S]*?LowCmdGetMotor\s*\([\s\S]*?LowCmdGetInhibitWriter\s*\([\s\S]*?CanTxCachedCmdAuthorized\s*\([\s\S]*?CanTxForceDisabledCmd\s*\(' -or
        $content -notmatch 'CanTxRecheckCommandAuthority\s*\(\s*actuator_id\s*,\s*&cmd\s*,\s*&have_cmd\s*,\s*&flags\s*\)[\s\S]{0,300}?CanTxProcessAxis\s*\(') {
        Add-CheckError "${repoPath}: every protocol path must recheck command generation and inhibit authority immediately before dispatch."
    }
    $cacheClearBody = [regex]::Match(
        $content,
        'static\s+void\s+CanTxClearCmdCache\s*\([^;]*\)\s*\{[\s\S]*?(?=\r?\nstatic\s+void\s+CanTxOutputGateSync)'
    ).Value
    if ($content -notmatch 'static\s+CanTxCmdExpiryLatch\s+s_can_tx_cmd_expiry_latch\s*\[\s*MotorCount\s*\]' -or
        [regex]::Matches($content, 'CanTxCmdExpiryLatchCheck\s*\(').Count -lt 2 -or
        $cacheClearBody -match 'cmd_expiry_latch') {
        Add-CheckError "${repoPath}: expired command generations must stay latched across full millisecond-tick wrap."
    }
    if ($content -notmatch 'static\s+CanTxCmdUnlockBarrier\s+s_can_tx_cmd_unlock_barrier\s*\[\s*MotorCount\s*\]' -or
        $content -notmatch 'CanTxCmdUnlockBarrierCapture\s*\(' -or
        $content -notmatch 'CanTxCmdPublishedAfterUnlock\s*\(' -or
        $content -notmatch 's_can_tx_unlock_barrier_pending\s*=\s*1u') {
        Add-CheckError "${repoPath}: unlocking must reject every command generation that existed before Lifecycle became active."
    }

    $n6014bPath = Join-Path $script:RepoRoot $n6014bRepoPath
    if (-not (Test-Path -LiteralPath $n6014bPath -PathType Leaf)) {
        Add-CheckError "Missing N6014b transport source: $n6014bRepoPath"
    }
    else {
        $n6014bContent = Get-Content -LiteralPath $n6014bPath -Raw -Encoding UTF8
        if ($n6014bContent -notmatch 'N6014bMotorSendActuator\s*\([^;{]*const\s+MotorCmd\s*\*\s*cmd' -or
            $n6014bContent -notmatch 'N6014bBuildCmdFromActuator\s*\([^;{]*const\s+MotorCmd\s*\*\s*cmd') {
            Add-CheckError "${n6014bRepoPath}: CanTx command snapshot must flow unchanged into the N6014b encoder."
        }
        if ($n6014bContent -notmatch 'static\s+uint8_t\s+N6014bActiveFrameAllowed[\s\S]*?RobotSafetyOutputLocked\s*\([\s\S]*?LowCmdGetMotor\s*\([\s\S]*?LowCmdGetInhibitWriter\s*\([\s\S]*?LowCmdSnapshotAuthorized\s*\(' -or
            $n6014bContent -notmatch 'taskENTER_CRITICAL\s*\(\s*\)[\s\S]{0,500}?N6014bActiveFrameAllowed[\s\S]{0,500}?N6014bTxStart[\s\S]{0,120}?taskEXIT_CRITICAL\s*\(\s*\)[\s\S]{0,180}?N6014bTxWait') {
            Add-CheckError "${n6014bRepoPath}: final authority, active/LOCK selection and non-blocking UART start must be linearized before waiting."
        }
        if ($n6014bContent -notmatch 'if\s*\(\s*cmd_mode\s*==\s*MotorModeDisable\s*\)\s*\{\s*return\s+1u\s*;\s*\}[\s\S]{0,250}?\*mode\s*=\s*N6014B_MODE_FOC\s*;') {
            Add-CheckError "${n6014bRepoPath}: Disable must remain in protocol LOCK mode and may not enter FOC."
        }
    }

    $unitreePath = Join-Path $script:RepoRoot $unitreeRepoPath
    if (-not (Test-Path -LiteralPath $unitreePath -PathType Leaf)) {
        Add-CheckError "Missing Unitree transport source: $unitreeRepoPath"
    }
    else {
        $unitreeContent = Get-Content -LiteralPath $unitreePath -Raw -Encoding UTF8
        if ($unitreeContent -notmatch 'UnitreeMotorBuildTxFrame\s*\(\s*&safe_frame[\s\S]{0,180}?UNITREE_MOTOR_MODE_BRAKE' -or
            $unitreeContent -notmatch 'static\s+uint8_t\s+UnitreeMotorActiveFrameAllowed[\s\S]*?RobotSafetyOutputLocked\s*\([\s\S]*?LowCmdGetMotor\s*\([\s\S]*?LowCmdGetInhibitWriter\s*\([\s\S]*?UnitreeMotorCmdSnapshotAllowed\s*\(' -or
            $unitreeContent -notmatch 'taskENTER_CRITICAL\s*\(\s*\)[\s\S]{0,500}?UnitreeMotorActiveFrameAllowed[\s\S]{0,500}?UnitreeMotorStartFrame[\s\S]{0,120}?taskEXIT_CRITICAL\s*\(\s*\)[\s\S]{0,180}?UnitreeMotorWaitFrame') {
            Add-CheckError "${unitreeRepoPath}: final authority must select a prebuilt BRAKE frame or start the active frame before waiting outside the critical section."
        }
    }

    $rs485BspPath = Join-Path $script:RepoRoot $rs485BspRepoPath
    if (-not (Test-Path -LiteralPath $rs485BspPath -PathType Leaf)) {
        Add-CheckError "Missing H7 RS485 BSP source: $rs485BspRepoPath"
    }
    else {
        $rs485BspContent = Get-Content -LiteralPath $rs485BspPath -Raw -Encoding UTF8
        $usart2ErrorBody = [regex]::Match($rs485BspContent, 'static\s+void\s+BspUsart2DispatchError[\s\S]*?(?=\r?\nstatic\s+void)').Value
        $usart3ErrorBody = [regex]::Match($rs485BspContent, 'static\s+void\s+BspUsart3DispatchError[\s\S]*?(?=\r?\nstatic\s+void)').Value
        if ($rs485BspContent -match 'int\s+BspUsart[23]Tx\s*\(' -or
            $rs485BspContent -notmatch 'HAL_UART_Transmit_IT\s*\(' -or
            $rs485BspContent -notmatch 'HAL_UART_TxCpltCallback[\s\S]*?BspRs485TxSignalFromIsr' -or
            $usart2ErrorBody -match 'BspRs485TxSignalFromIsr' -or
            $usart3ErrorBody -match 'BspRs485TxSignalFromIsr') {
            Add-CheckError "${rs485BspRepoPath}: RS485 TX must complete by UART TC; blocking TX and RX-error false completion may not return."
        }
        if ($rs485BspContent -notmatch 'BspRs485SetBaudrate[\s\S]{0,600}?gState\s*!=\s*HAL_UART_STATE_READY[\s\S]{0,180}?return\s+\(int\)HAL_BUSY[\s\S]{0,180}?HAL_UART_Abort\s*\(') {
            Add-CheckError "${rs485BspRepoPath}: baud changes must reject TX-busy ports before HAL abort can truncate a committed frame."
        }
    }
}

function Test-ManualInputSnapshotBoundaries {
    Write-Host "[check] manual input snapshot boundaries"

    $snapshotRepoPath = "shared\application\input\ManualInputSnapshot.h"
    $manualHeaderRepoPath = "shared\application\input\ManualInput.h"
    $controlHeaderRepoPath = "shared\application\input\ControlInput.h"
    $manualRepoPath = "shared\application\input\ManualInput.c"
    $controlRepoPath = "shared\application\input\ControlInput.c"
    foreach ($repoPath in @($snapshotRepoPath, $manualHeaderRepoPath, $controlHeaderRepoPath, $manualRepoPath, $controlRepoPath)) {
        if (-not (Test-Path -LiteralPath (Join-Path $script:RepoRoot $repoPath) -PathType Leaf)) {
            Add-CheckError "Missing manual input snapshot file: $repoPath"
            return
        }
    }

    $snapshotHeader = Get-Content -LiteralPath (Join-Path $script:RepoRoot $snapshotRepoPath) -Raw -Encoding UTF8
    $manualHeader = Get-Content -LiteralPath (Join-Path $script:RepoRoot $manualHeaderRepoPath) -Raw -Encoding UTF8
    $controlHeader = Get-Content -LiteralPath (Join-Path $script:RepoRoot $controlHeaderRepoPath) -Raw -Encoding UTF8
    $manualContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $manualRepoPath) -Raw -Encoding UTF8
    $controlContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $controlRepoPath) -Raw -Encoding UTF8

    if ($snapshotHeader -notmatch '#include\s+"ManualInput\.h"' -or
        $snapshotHeader -notmatch '#include\s+"ControlInput\.h"') {
        Add-CheckError "${snapshotRepoPath}: aggregate header must own the one-way ManualInput/ControlInput dependency."
    }
    if ($manualHeader -match '#include\s+"ManualInputSnapshot\.h"' -or
        $controlHeader -match '#include\s+"ManualInputSnapshot\.h"') {
        Add-CheckError "ManualInput.h and ControlInput.h must not include ManualInputSnapshot.h back and form a header cycle."
    }
    foreach ($field in @(
            "manual", "control", "semantics", "activeMask", "sourceTickMs", "publishTickMs", "readTickMs",
            "sourceAgeMs", "sourceTimeoutMs", "publishSeq", "sourceSeq", "switchSeq",
            "semanticsSeq", "actionSeq", "authoritySeq", "activeSource", "online", "dataValid",
            "mixMode", "sourceProtocol", "sourceFlags"
        )) {
        if ($snapshotHeader -notmatch "\b$field\b") {
            Add-CheckError "${snapshotRepoPath}: snapshot is missing '$field'."
        }
    }
    if ($snapshotHeader -notmatch 'MANUAL_INPUT_SNAPSHOT_STACK_BUDGET_BYTES\s+128u' -or
        $snapshotHeader -notmatch 'sizeof\s*\(\s*ManualInputSnapshot\s*\)\s*<=\s*MANUAL_INPUT_SNAPSHOT_STACK_BUDGET_BYTES') {
        Add-CheckError "${snapshotRepoPath}: frame-owned input snapshot must keep its explicit 128-byte task-stack budget."
    }

    $buildBody = [regex]::Match(
        $controlContent,
        'void\s+ControlInputBuild\s*\([\s\S]*?(?=\r?\nuint8_t\s+ControlInputSwitchPosToRaw\s*\()'
    ).Value
    if ([string]::IsNullOrWhiteSpace($buildBody)) {
        Add-CheckError "${controlRepoPath}: cannot find pure ControlInputBuild implementation."
    }
    elseif ($buildBody -match '\bg_config\b|EnterCritical|ExitCritical|ManualInputSnapshotRead') {
        Add-CheckError "${controlRepoPath}: ControlInputBuild must stay pure and may not read globals, lock, or read a snapshot."
    }
    foreach ($legacyInputApi in @(
            'ManualInputGetCurrentRc',
            'ManualInputGetCurrentCopy',
            'ManualInputGetActiveSource',
            'get_remote_control_point',
            'remote_control_get_active_source',
            'remote_control_init',
            'remote_control_on_sbus_frame',
            'remote_control_set_rc',
            'remote_control_set_rc_source',
            'remote_control_refresh',
            'RC_data_is_error',
            'slove_RC_lost',
            'slove_data_error',
            'ManualInputUpdateSource',
            'ManualInputUpdateSourceMeta'
        )) {
        if ($manualContent -match ("\b" + $legacyInputApi + "\s*\(") -or
            $manualHeader -match ("\b" + $legacyInputApi + "\s*\(")) {
            Add-CheckError "${manualRepoPath}: obsolete parallel input reader '$legacyInputApi' must stay removed."
        }
    }
    foreach ($legacyControlApi in @(
            'ControlInputUpdateFromManualInput',
            'ControlInputGetState',
            'ControlInputGetCopy',
            'ControlInputAxis',
            'ControlInputSwitch',
            'input_update_from_rc',
            'input_get',
            'input_get_copy',
            'input_axis',
            'input_switch'
        )) {
        if ($controlContent -match ("\b" + $legacyControlApi + "\s*\(") -or
            $controlHeader -match ("\b" + $legacyControlApi + "\s*\(")) {
            Add-CheckError "${controlRepoPath}: obsolete parallel control reader '$legacyControlApi' must stay removed."
        }
    }

    $readBody = [regex]::Match(
        $manualContent,
        'uint8_t\s+ManualInputSnapshotRead\s*\([\s\S]*?(?=\r?\nstatic\s+void\s+ManualInputMarkDirty\s*\()'
    ).Value
    if ([string]::IsNullOrWhiteSpace($readBody)) {
        Add-CheckError "${manualRepoPath}: cannot find ManualInputSnapshotRead."
    }
    else {
        foreach ($forbidden in @("ManualInputRefreshIfNeeded", "WatchUpdateRcSnapshot", "SdLogWrite")) {
            if ($readBody -match [regex]::Escape($forbidden)) {
                Add-CheckError "${manualRepoPath}: snapshot read must not trigger '$forbidden'."
            }
        }
        if ($manualContent -match 'readers\s*\[' -or
            $manualContent -match 'ManualInputCompressSafe\s*\(') {
            Add-CheckError "${manualRepoPath}: snapshot publication must keep one inactive writer bank and may not restore reader pins or lossy compression."
        }
        $actionSyncIndex = $readBody.IndexOf('ManualInputActionAuthoritySyncLocked')
        $controlSyncIndex = $readBody.IndexOf('ManualInputControlAuthoritySyncLocked')
        $finalCriticalExitIndex = $readBody.LastIndexOf('ManualInputExitCritical(critical)')
        if ($readBody -notmatch 'ManualInputEnterCritical\s*\(\s*\)[\s\S]{0,180}?ManualInputStore\.active_bank' -or
            $readBody -notmatch 'excluded_mask\[bank\][\s\S]*?ManualInputInvalidatedMaskLocked\s*\(\s*bank\s*\)' -or
            $readBody -notmatch '\*out\s*=\s*ManualInputStore\.bank\[bank\]' -or
            $readBody -notmatch 'ManualInputBuildSnapshot\s*\(\s*ManualInputStore\.source\[bank\][\s\S]{0,500}?ManualInputStore\.board_key_down\[bank\]' -or
            $actionSyncIndex -lt 0 -or $controlSyncIndex -le $actionSyncIndex -or
            $finalCriticalExitIndex -le $controlSyncIndex) {
            Add-CheckError "${manualRepoPath}: one critical section must linearize bank selection, full copy, invalidation fallback and action/control authority generations."
        }
        if ($manualContent -notmatch 'ManualInputSrcState\s+source\s*\[\s*MANUAL_INPUT_SNAPSHOT_BANK_COUNT\s*\]\s*\[\s*MANUAL_INPUT_SRC_MAX\s*\+\s*1u\s*\]' -or
            $manualContent -notmatch 'uint32_t\s+invalidate_gen\s*;' -or
            $manualContent -notmatch 'ManualInputCrsfState\s+crsf\s*\[\s*MANUAL_INPUT_SNAPSHOT_BANK_COUNT\s*\]' -or
            $manualContent -notmatch 'uint16_t\s+channel\s*\[\s*MANUAL_INPUT_CRSF_CHANNEL_COUNT\s*\]' -or
            $manualContent -notmatch 'uint32_t\s+excluded_mask\s*\[\s*MANUAL_INPUT_SNAPSHOT_BANK_COUNT\s*\]') {
            Add-CheckError "${manualRepoPath}: each bank must retain per-source evidence, CRSF raw mapping evidence, exact invalidation generations and an exclusion latch."
        }
    }

    $refreshBody = [regex]::Match(
        $manualContent,
        'static\s+void\s+ManualInputRefreshIfNeeded\s*\([^;]*\)\s*\{[\s\S]*?(?=\r?\nstatic\s+void\s+ManualInputRefreshTimerCallback\s*\([^;]*\)\s*\{)'
    ).Value
    if ([string]::IsNullOrWhiteSpace($refreshBody)) {
        Add-CheckError "${manualRepoPath}: cannot find bounded snapshot publisher."
    }
    else {
        if ($manualContent -notmatch 'static\s+ManualInputRefreshWorkspace\s+ManualInputWorkspace\s*;' -or
            $refreshBody -match 'ManualInputSrcState\s+src_snapshot\s*\[|ManualInputSnapshot\s+(candidate|published_snapshot)\s*;') {
            Add-CheckError "${manualRepoPath}: timer-daemon refresh buffers must stay in the single-writer static workspace."
        }
        if ($refreshBody -notmatch 'ManualInputDirtySeq\s*==\s*dirty_seq[\s\S]*?memcmp\s*\(\s*&g_config\.manual_input[\s\S]*?memcmp\s*\(\s*&g_config\.input' -or
            $refreshBody -notmatch 'ManualInputDirtySeq\s*==\s*dirty_seq[\s\S]*?ManualInputStore\.active_bank\s*=\s*inactive_bank') {
            Add-CheckError "${manualRepoPath}: dirty generation and both config blocks must be validated immediately before bank flip."
        }
        if ($manualContent -notmatch 'out->semantics\s*=\s*manual_cfg->semantics' -or
            $manualContent -notmatch 'ManualInputWorkspace\.candidate\.semanticsSeq\s*=\s*1u' -or
            $manualContent -notmatch 'ManualInputConfig\s+manual_config\s*\[\s*MANUAL_INPUT_SNAPSHOT_BANK_COUNT\s*\]' -or
            $manualContent -notmatch 'input_config_t\s+input_config\s*\[\s*MANUAL_INPUT_SNAPSHOT_BANK_COUNT\s*\]' -or
            $refreshBody -notmatch 'ManualInputWorkspace\.candidate\.semanticsSeq\s*=[\s\S]{0,700}?memcmp\s*\(\s*&ManualInputWorkspace\.manualConfig[\s\S]{0,250}?ManualInputStore\.manual_config\[current_bank\][\s\S]{0,300}?memcmp\s*\(\s*&ManualInputWorkspace\.inputConfig[\s\S]{0,250}?ManualInputStore\.input_config\[current_bank\][\s\S]{0,300}?ManualInputSeqNext\s*\(\s*semantics_seq\s*\)[\s\S]{0,100}?semantics_seq') {
            Add-CheckError "${manualRepoPath}: input interpretation generation must start at 1 and advance when either frozen input config block changes."
        }
        if ($refreshBody -notmatch 'ManualInputExpireSourcesLocked' -or
            $manualContent -notmatch 'manual_src\[source\]\.update_seq\s*==\s*src_state\[source\]\.update_seq[\s\S]*?manual_src\[source\]\.valid\s*=\s*0u') {
            Add-CheckError "${manualRepoPath}: confirmed stale sources must be invalidated by matching source generation."
        }
        $logIndex = $refreshBody.IndexOf('ManualInputLogSourceSwitch')
        $busyClearIndex = $refreshBody.LastIndexOf('ManualInputRefreshBusy = 0u')
        if ($logIndex -lt 0 -or $busyClearIndex -lt 0 -or $logIndex -gt $busyClearIndex) {
            Add-CheckError "${manualRepoPath}: writer busy must cover source-switch logging side effects."
        }
        if ($manualContent -match '#include\s+"Watch\.h"|WatchUpdateRcSnapshot\s*\(') {
            Add-CheckError "${manualRepoPath}: input publication must not push directly into the diagnostics layer."
        }
    }

    $updateSourceBody = [regex]::Match(
        $manualContent,
        'static\s+void\s+ManualInputUpdateSourceDetail\s*\([\s\S]*?(?=\r?\nvoid\s+ManualInputInvalidateSource\s*\()'
    ).Value
    if ($updateSourceBody -notmatch 'if\s*\(\s*source\s*==\s*MANUAL_INPUT_SRC_DBUS\s*&&\s*source_valid\s*!=\s*0u\s*\)[\s\S]{0,160}?DetectHook\s*\(\s*DBUS_TOE\s*\)') {
        Add-CheckError "${manualRepoPath}: only valid physical DBUS frames may refresh DBUS_TOE after aggregate-input consumers migrate."
    }
    $onlineValidityAssignmentCount = ([regex]::Matches(
        $manualContent,
        'out->online\s*=\s*\(uint8_t\)\([^;]*?dataValid\s*!=\s*0u\s*\)')).Count
    if ($updateSourceBody -notmatch 'source_copy\s*=\s*\*rc[\s\S]{0,180}?source_valid\s*=\s*ManualInputStateValid\s*\(\s*&source_copy\s*\)' -or
        $updateSourceBody -notmatch 'manual_src\[source\]\.rc\s*=\s*source_copy' -or
        $updateSourceBody -notmatch 'manual_src\[source\]\.valid\s*=\s*source_valid' -or
        $onlineValidityAssignmentCount -lt 2) {
        Add-CheckError "${manualRepoPath}: invalid source frames must leave arbitration immediately, and published/read online must imply same-generation data validity."
    }
    $imageInputRepoPath = "shared\application\input\ImageRemoteLink.c"
    $imageInputContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $imageInputRepoPath) -Raw -Encoding UTF8
    if ([regex]::Matches($imageInputContent, 'ManualInputUpdateImageSource\s*\(').Count -ne 2 -or
        $manualContent -notmatch 'out->sourceProtocol\s*=\s*src_state\[selected\]\.protocol' -or
        $manualContent -notmatch 'merged_flags\s*\|=\s*ManualInputSourceFlags\s*\(' -or
        $manualContent -notmatch 'out->sourceFlags\s*=\s*\(mix_mode\s*==\s*MANUAL_INPUT_MIX_MERGE\)') {
        Add-CheckError "Manual input must publish Image protocol/business metadata in the same generation as raw and mapped input."
    }
    if ($manualContent -match '\bRC_abs\s*\(' -or
        $manualContent -match 'out->manual\.(?:key\.v|mouse\.press_[lr])\s*\|=' -or
        $manualContent -notmatch 'ManualInputResolveSource\s*\(\s*&src_state\[selected\]\s*,\s*crsf\s*,\s*input_cfg\s*,\s*&out->manual\s*\)' -or
        $manualContent -notmatch 'contributor_mask\s*\|=\s*snapshot->activeMask\s*&[\s\S]{0,100}?MANUAL_INPUT_SRC_IMAGE') {
        Add-CheckError "${manualRepoPath}: MERGE may union typed Image business flags, but the representative source must exclusively own axes, switches and generic key/mouse input."
    }
    if ($manualContent -notmatch 'ManualInputUpdateImageSource[\s\S]{0,260}?rc\s*==\s*NULL[\s\S]{0,260}?ManualInputInvalidateSource\s*\(\s*MANUAL_INPUT_SRC_IMAGE\s*\)' -or
        $manualContent -notmatch 'ManualInputUpdateElrsChannels[\s\S]{0,260}?decoded\s*==\s*NULL\s*\|\|\s*raw\s*==\s*NULL[\s\S]{0,220}?ManualInputInvalidateSource\s*\(\s*MANUAL_INPUT_SRC_ELRS\s*\)') {
        Add-CheckError "${manualRepoPath}: typed source writers must revoke their old command when their payload is missing."
    }
    if ($refreshBody -notmatch 'force\s*==\s*0u[\s\S]{0,220}?ManualInputRefreshDirty\s*==\s*0u[\s\S]{0,700}?BspKeyReadRawDown\s*\(\s*\)\s*==\s*frozen_board_key[\s\S]{0,120}?return') {
        Add-CheckError "${manualRepoPath}: an unchanged input system must not rebuild and flip snapshot banks at a fixed timer rate."
    }
    if ($manualHeader -notmatch 'MANUAL_INPUT_SOURCE_RAW_BTN_R' -or
        $manualHeader -notmatch 'uint8_t\s+rawSwitch1' -or
        $manualContent -notmatch 'uint8_t\s+raw_switch1\s*;' -or
        $manualContent -notmatch 'ManualInputApplySourceMapping\s*\(\s*&out->manual\s*,\s*&src_state\[selected\]\s*,\s*manual_cfg\s*\)' -or
        $manualContent -notmatch 'ManualInputMapVt13Switch1\s*\(\s*source->raw_switch1\s*,\s*config\s*\)' -or
        $manualContent -notmatch 'ManualInputMapVt13Switch2\s*\(\s*source->raw_flags\s*,\s*config\s*\)') {
        Add-CheckError "${manualRepoPath}: VT13 raw switches must be stored with the source and remapped from the frozen snapshot config."
    }

    $vt13ParserBody = [regex]::Match(
        $imageInputContent,
        'static\s+void\s+ImageRemoteLinkHandleVt13Frame\s*\([^;]*\)\s*\{[\s\S]*?(?=\r?\nstatic\s+bool_t\s+ImageRemoteLinkTryDecodeCustomRc\s*\()'
    ).Value
    $customParserBody = [regex]::Match(
        $imageInputContent,
        'static\s+bool_t\s+ImageRemoteLinkTryDecodeCustomRc\s*\([^;]*\)\s*\{[\s\S]*?(?=\r?\nstatic\s+int16_t\s+ImageRemoteLinkScaleAxis\s*\()'
    ).Value
    if ([string]::IsNullOrWhiteSpace($vt13ParserBody) -or
        $vt13ParserBody -match '\bg_config\b|ImageRemoteLinkMapVt13Switch' -or
        $vt13ParserBody -notmatch 'rc\.rc\.s\[0\]\s*=\s*\(char\)RC_SW_UP' -or
        $vt13ParserBody -notmatch 'rc\.rc\.s\[1\]\s*=\s*\(char\)RC_SW_UP' -or
        $vt13ParserBody -notmatch 'ImageRemoteRawFlags\s*\(\s*&state\s*\)\s*,\s*vt13_switch1') {
        Add-CheckError "${imageInputRepoPath}: VT13 parser must forward raw switches without reading live config."
    }
    if ([string]::IsNullOrWhiteSpace($customParserBody) -or
        $customParserBody -notmatch 'SDLOG_MANUAL_INPUT_PROTO_IMAGE_CUSTOM' -or
        $customParserBody -notmatch 'ImageRemoteRawFlags\s*\(\s*&state\s*\)\s*,\s*0u') {
        Add-CheckError "${imageInputRepoPath}: custom Image protocol must forward IMAGE_CUSTOM metadata through the same source update."
    }
    if ($imageInputContent -notmatch 'state->btn_r[\s\S]{0,100}MANUAL_INPUT_SOURCE_RAW_BTN_R') {
        Add-CheckError "${imageInputRepoPath}: VT13 right button must remain part of the frozen raw source metadata."
    }

    foreach ($testRepoPath in @(
            "tools\TestManualInputSnapshot.ps1",
            "tools\TestImageRemoteInput.ps1",
            "tools\tests\ManualInputSnapshotRegression.c",
            "tools\tests\ImageRemoteInputRegression.c",
            "tools\tests\manual-input-stubs\FreeRTOS.h",
            "tools\tests\manual-input-stubs\BspUsart.h",
            "tools\tests\manual-input-stubs\task.h",
            "tools\tests\manual-input-stubs\timers.h"
        )) {
        if (-not (Test-Path -LiteralPath (Join-Path $script:RepoRoot $testRepoPath) -PathType Leaf)) {
            Add-CheckError "Missing manual input snapshot regression file: $testRepoPath"
        }
    }

    $manualRegressionRepoPath = "tools\tests\ManualInputSnapshotRegression.c"
    $manualRegressionPath = Join-Path $script:RepoRoot $manualRegressionRepoPath
    if (Test-Path -LiteralPath $manualRegressionPath -PathType Leaf) {
        $manualRegressionContent = Get-Content -LiteralPath $manualRegressionPath -Raw -Encoding UTF8
        foreach ($requiredPattern in @(
                'TestSemanticsSameGeneration\s*\(',
                'TestDeferredInvalidationBarrier\s*\(',
                'TestPinnedSourceTimeoutFallback\s*\(',
                'TestMergeDegradesPerSource\s*\(',
                'TestStickyAuthorityAndExplicitReclaim\s*\(',
                'TestDerivedMultiStageAuthorityFallback\s*\(',
                'TestMergeAuxiliaryActionDoesNotChangeControlAuthority\s*\(',
                'TestCrsfMappingRefreshUsesFrozenRaw\s*\(',
                'TestIdleTimerDoesNotRepublish\s*\(',
                'TestTypedWriterNullRevokesSource\s*\(',
                'ManualInputRefreshBusy\s*=\s*1u',
                'ManualInputStore\.excluded_mask\[ManualInputStore\.active_bank\]',
                'snapshot\.manual\.mouse\.x\s*==\s*INT16_MIN',
                'snapshot\.semanticsSeq\s*==\s*1u',
                'snapshot\.actionSeq',
                'snapshot\.authoritySeq',
                'g_config\.manual_input\.semantics\s*=\s*updated_semantics',
                'g_config\.input\.sw\[INPUT_SW_GIMBAL_MODE\]\.invert\s*=\s*1u',
                'semanticsSeq\s*=\s*UINT32_MAX',
                'snapshot\.semanticsSeq\s*==\s*ManualInputSeqNext\s*\(\s*semantics_seq\s*\)'
            )) {
            if ($manualRegressionContent -notmatch $requiredPattern) {
                Add-CheckError "${manualRegressionRepoPath}: frozen semantics generation regression is missing '$requiredPattern'."
            }
        }
    }

    $imageRegressionRepoPath = "tools\tests\ImageRemoteInputRegression.c"
    $imageRegressionPath = Join-Path $script:RepoRoot $imageRegressionRepoPath
    if (Test-Path -LiteralPath $imageRegressionPath -PathType Leaf) {
        $imageRegressionContent = Get-Content -LiteralPath $imageRegressionPath -Raw -Encoding UTF8
        foreach ($requiredPattern in @(
                '#include\s+"ImageRemoteLink\.c"',
                'BSP_AUX_LINK_RXEVENT_IDLE',
                'TestBuildCustomFrame\s*\(',
                'TestBuildVt13Frame\s*\(',
                'MANUAL_INPUT_PROTOCOL_CRSF',
                'MANUAL_INPUT_PROTOCOL_IMAGE_CUSTOM',
                'ManualInputStore\.ready\s*=\s*0u',
                'TestInvalidCustomSwitchInvalidatesVt13\s*\(',
                'TestInvalidCustomBusinessFields\s*\(',
                'TestInterruptFallbackDefersParsingToTask\s*\(',
                'stats\.crc_error_count\s*==\s*1u',
                'snapshot\.sourceProtocol\s*==\s*MANUAL_INPUT_PROTOCOL_NONE',
                'TestParserArrivalDuringPublish\s*\('
            )) {
            if ($imageRegressionContent -notmatch $requiredPattern) {
                Add-CheckError "${imageRegressionRepoPath}: real ImageRemote regression is missing '$requiredPattern'."
            }
        }
    }

    if ($manualHeader -notmatch 'ManualInputInvalidateSource\s*\(' -or
        $imageInputContent -notmatch 'ManualInputInvalidateSource\s*\(\s*MANUAL_INPUT_SRC_IMAGE\s*\)' -or
        $imageInputContent -match 'return\s+RC_SW_DOWN\s*;') {
        Add-CheckError "${imageInputRepoPath}: a framed Image packet with invalid switches must revoke IMAGE instead of being sanitized into an action position."
    }

    $elrsRepoPath = "shared\application\input\ElrsTask.c"
    $elrsContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $elrsRepoPath) -Raw -Encoding UTF8
    $elrsItBody = [regex]::Match(
        $elrsContent,
        'void\s+ElrsLinkOnItByte\s*\([^;]*\)\s*\{[\s\S]*?(?=\r?\nbool_t\s+ElrsLinkOnUartError\s*\()').Value
    $imageItBody = [regex]::Match(
        $imageInputContent,
        'void\s+ImageRemoteLinkOnItByte\s*\([^;]*\)\s*\{[\s\S]*?(?=\r?\nuint8_t\s+ImageRemoteLinkOnUartError\s*\()').Value
    $elrsStopBody = [regex]::Match(
        $elrsContent,
        'void\s+ElrsLinkStop\s*\([^;]*\)\s*\{[\s\S]*?(?=\r?\nvoid\s+ElrsLinkRxStart\s*\()').Value
    $imageStopBody = [regex]::Match(
        $imageInputContent,
        'void\s+ImageRemoteLinkStop\s*\([^;]*\)\s*\{[\s\S]*?(?=\r?\nvoid\s+ImageRemoteLinkPoll\s*\()').Value
    if ([string]::IsNullOrWhiteSpace($elrsItBody) -or
        $elrsItBody -notmatch 'ElrsLinkRx\.itRing\[head\]\s*=\s*b' -or
        $elrsItBody -match 'ElrsLinkOnByte\s*\(|ManualInput\w*\s*\(|SdLogWrite\s*\(' -or
        [string]::IsNullOrWhiteSpace($imageItBody) -or
        $imageItBody -notmatch 'ImageRemoteRx\.itRing\[head\]\s*=\s*value' -or
        $imageItBody -match 'ImageRemoteLinkFeedByte\s*\(|ManualInput\w*\s*\(|SdLogWrite\s*\(') {
        Add-CheckError "Image/ELRS byte interrupts may only enqueue bytes; parsing, logging and input publication belong to task context."
    }
    if ($elrsStopBody.IndexOf('ElrsLinkDmaActive = 0u') -lt 0 -or
        $elrsStopBody.IndexOf('ManualInputInvalidateSource') -le $elrsStopBody.IndexOf('ElrsLinkDmaActive = 0u') -or
        $imageStopBody.IndexOf('ImageRemoteDmaActive = 0u') -lt 0 -or
        $imageStopBody.IndexOf('ImageRemoteInvalidate') -le $imageStopBody.IndexOf('ImageRemoteDmaActive = 0u') -or
        $elrsContent -notmatch 'ElrsLinkDmaActive\s*!=\s*0u\s*&&[\s\S]{0,120}?ELRS_LINK_NOTIFY_RESTART') {
        Add-CheckError "Image/ELRS stop must close the ISR/task publication gate before invalidating the source, and stale restart notifications may not reopen a stopped link."
    }
    if ($elrsContent -notmatch 'typedef\s+union[\s\S]{0,220}?dma\[ELRS_LINK_DMA_RX_BUF_SIZE\][\s\S]{0,120}?itRing\[ELRS_LINK_IT_RX_RING_SIZE\]' -or
        $imageInputContent -notmatch 'typedef\s+union[\s\S]{0,220}?dma\[IMAGE_REMOTE_DMA_RX_BUF_SIZE\][\s\S]{0,120}?itRing\[IMAGE_REMOTE_IT_RX_RING_SIZE\]' -or
        $elrsContent -notmatch 'BspAuxLinkRxItStart\s*\(' -or
        $imageInputContent -notmatch 'BspAuxLinkSetRxByteCb\s*\(\s*ImageRemoteLinkOnItByte\s*\)[\s\S]{0,280}?BspAuxLinkRxItStart\s*\(') {
        Add-CheckError "Image/ELRS DMA and byte-interrupt fallback must share mutually exclusive RX storage and keep a functional no-DMA start path."
    }
    if ($elrsContent -match '\bError_Handler\s*\(' -or
        $elrsContent -notmatch 'BspAuxLinkRxToIdleDmaStart[\s\S]{0,220}?WatchTaskError\s*\(\s*WATCH_TASK_ELRS\s*\)[\s\S]{0,160}?return\s*;' -or
        $elrsContent -notmatch 'ElrsLinkItRxOverflow[\s\S]{0,280}?ManualInputInvalidateSource\s*\(\s*MANUAL_INPUT_SRC_ELRS\s*\)' -or
        $imageInputContent -notmatch 'ImageRemoteItRxOverflow[\s\S]{0,320}?ImageRemoteInvalidate\s*\(\s*\)') {
        Add-CheckError "AUX link startup/overflow failures must invalidate only their source in task context and must never enter the global fatal path."
    }

    $configParamRepoPath = "shared\application\robot\ConfigParamList.inc"
    $configParamContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $configParamRepoPath) -Raw -Encoding UTF8
    if ($configParamContent -notmatch 'CONFIG_PARAM_U8_DEFAULT\(300,[^\r\n]*MANUAL_INPUT_SRC_AUTO' -or
        $configParamContent -notmatch 'CONFIG_PARAM_U8_DEFAULT\(301,[^\r\n]*MANUAL_INPUT_MIX_SELECT_STICKY[^\r\n]*MANUAL_INPUT_MIX_SELECT_STICKY' -or
        ([regex]::Matches($configParamContent, 'CONFIG_PARAM_U8_MAX\(31[0-6],[^\r\n]*15u')).Count -ne 7) {
        Add-CheckError "${configParamRepoPath}: manual source defaults must stay AUTO+STICKY and all seven CRSF mapping indices must be bounded to channel 15."
    }
    foreach ($configInputFile in (Get-ChildItem -LiteralPath (Join-Path $script:RepoRoot "Robotconfig") -Recurse -Filter "ConfigInput.inc" -File)) {
        $configInputContent = Get-Content -LiteralPath $configInputFile.FullName -Raw -Encoding UTF8
        if ($configInputContent -notmatch '\.active_source\s*=\s*MANUAL_INPUT_SRC_AUTO' -or
            $configInputContent -notmatch '\.mix_mode\s*=\s*MANUAL_INPUT_MIX_SELECT_STICKY') {
            Add-CheckError "$(Format-RepoPath $configInputFile.FullName): default manual arbitration must stay AUTO+STICKY."
        }
    }
    if ($manualContent -notmatch 'tail_retry[\s\S]{0,260}?force\s*!=\s*2u[\s\S]{0,120}?ManualInputRefreshIfNeeded\s*\(\s*2u\s*\)') {
        Add-CheckError "${manualRepoPath}: input arriving during source-switch logging must receive one bounded tail refresh."
    }

    $gimbalRuntimeRepoPath = "shared\application\gimbal\GimbalRuntimeHelpers.inc"
    $gimbalRuntimeContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $gimbalRuntimeRepoPath) -Raw -Encoding UTF8
    $autoAimBody = [regex]::Match(
        $gimbalRuntimeContent,
        'fast\.image_auto_aim_requested\s*=[\s\S]*?;').Value
    $shootRepoPath = "shared\application\shoot\Shoot.c"
    $shootContent = Get-SourceContentWithPrivateIncludes -Path (Join-Path $script:RepoRoot $shootRepoPath)
    $shootFrameBody = [regex]::Match(
        $shootContent,
        'static\s+void\s+ShootFrameInputCapture\s*\([^;]*\)\s*\{[\s\S]*?(?=\r?\n\})').Value
    if ($autoAimBody -notmatch 'MANUAL_INPUT_SOURCE_FLAG_AUTO_AIM' -or
        $autoAimBody -match 'activeSource\s*==\s*MANUAL_INPUT_SRC_IMAGE' -or
        $shootFrameBody -notmatch 'MANUAL_INPUT_SOURCE_FLAG_AUX_FIRE' -or
        $shootFrameBody -match 'activeSource\s*==\s*MANUAL_INPUT_SRC_IMAGE') {
        Add-CheckError "Gimbal/Shoot must consume normalized aggregate Image business flags even when MERGE has another representative source."
    }

    foreach ($insInputRepoPath in @(
            "boards\DjiAF427\bsp\InsTask.c",
            "boards\DjiCF407\bsp\InsTask.c",
            "boards\DmMc02H7\app\InsTask.c"
        )) {
        $insInputPath = Join-Path $script:RepoRoot $insInputRepoPath
        if (-not (Test-Path -LiteralPath $insInputPath -PathType Leaf)) {
            Add-CheckError "Missing IMU input consumer: $insInputRepoPath"
            continue
        }

        $insInputContent = Get-Content -LiteralPath $insInputPath -Raw -Encoding UTF8
        $bootAdjustBody = [regex]::Match(
            $insInputContent,
            'static\s+uint8_t\s+GyroZeroBootAdjustAllowedByInput\s*\([^)]*\)\s*\{[\s\S]*?(?=\r?\n\})'
        ).Value
        if ($insInputContent -notmatch '#include\s+"ManualInputSnapshot\.h"' -or
            $insInputContent -match '#include\s+"(?:ManualInput|ControlInput)\.h"' -or
            [regex]::Matches($insInputContent, 'ManualInputSnapshotRead\s*\(').Count -ne 1 -or
            [string]::IsNullOrWhiteSpace($bootAdjustBody) -or
            $bootAdjustBody -notmatch 'ManualInputSnapshot\s+input\s*;' -or
            $bootAdjustBody -notmatch 'ManualInputSnapshotRead\s*\(\s*&input\s*\)\s*==\s*0u[\s\S]{0,100}?input\.online\s*==\s*0u[\s\S]{0,180}?return\s+1u' -or
            $bootAdjustBody -notmatch 'input\.control\.sw\s*\[\s*INPUT_SW_GIMBAL_MODE\s*\][\s\S]{0,100}?input\.semantics\.GimbalSafePos' -or
            $bootAdjustBody -notmatch 'input\.control\.sw\s*\[\s*INPUT_SW_CHASSIS_MODE\s*\][\s\S]{0,100}?input\.semantics\.ChassisSafePos') {
            Add-CheckError "${insInputRepoPath}: automatic gyro adjustment must keep offline stationary correction and use one online snapshot with frozen control semantics."
        }
        if ($insInputContent -match '\bDBUS_TOE\b|ManualInputGet\w*\s*\(|ControlInput(?:Get\w*|Axis|Switch)\s*\(|\binput_(?:get|axis|switch)\w*\s*\(|\bg_config\s*\.\s*manual_input\s*\.\s*semantics\b|\bMANUAL_INPUT_SRC_\w+\b') {
            Add-CheckError "${insInputRepoPath}: IMU input permission must stay source-neutral and may not return to legacy/live input reads."
        }
    }

    $wheellegRepoPath = "shared\application\wheelleg\WheelLegMitTask.c"
    $wheellegPath = Join-Path $script:RepoRoot $wheellegRepoPath
    $wheellegContent = Get-SourceContentWithPrivateIncludes -Path $wheellegPath
    if (([regex]::Matches($wheellegContent, 'ManualInputSnapshotRead\s*\(')).Count -ne 1 -or
        $wheellegContent -notmatch 'ManualInputSnapshot\s+manual_input_snapshot\s*;' -or
        $wheellegContent -notmatch 'ManualInputSnapshotRead\s*\(\s*&frame->manual_input_snapshot\s*\)' -or
        $wheellegContent -notmatch 'frame->manual_input_snapshot\.online' -or
        $wheellegContent -notmatch 'input->control\.axis' -or
        $wheellegContent -notmatch 'input->control\.sw' -or
        $wheellegContent -notmatch '&frame->manual_input_snapshot\.semantics') {
        Add-CheckError "${wheellegRepoPath}: WheelLeg MIT must read one frame-owned aggregate snapshot and use its online/control/semantics generation throughout the frame."
    }
    foreach ($forbiddenPattern in @(
            '\bDBUS_TOE\b',
            'toe_is_error\s*\(\s*DBUS_TOE',
            'ManualInputGet\w*\s*\(',
            'ControlInputGet(?:Copy|State)\s*\(',
            'ControlInput(?:Axis|Switch)\s*\(',
            '\bg_config\.input\b',
            '\bg_config\.manual_input\.semantics\b',
            'static\s+(?:const\s+)?ManualInputSnapshot\s*\*'
        )) {
        if ($wheellegContent -match $forbiddenPattern) {
            Add-CheckError "${wheellegRepoPath}: WheelLeg MIT frame input must not use legacy/source-specific/live-config API '$forbiddenPattern'."
        }
    }

    foreach ($requiredServiceInputPath in @(
            "shared\application\arm\ArmInputPolicy.h",
            "shared\application\chassis\ChassisInputPolicy.h",
            "shared\application\gimbal\GimbalInputPolicy.h",
            "shared\application\services\servo\ServoInputPolicy.h",
            "tools\TestServiceInputPolicy.ps1",
            "tools\tests\ServiceInputPolicyRegression.c"
        )) {
        if (-not (Test-Path -LiteralPath (Join-Path $script:RepoRoot $requiredServiceInputPath) -PathType Leaf)) {
            Add-CheckError "Missing service input regression asset: $requiredServiceInputPath"
        }
    }

    $armPolicyRepoPath = "shared\application\arm\ArmInputPolicy.h"
    $armPolicyContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $armPolicyRepoPath) -Raw -Encoding UTF8
    $armTaskRepoPath = "shared\application\arm\ArmTask.c"
    $armTaskContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $armTaskRepoPath) -Raw -Encoding UTF8
    if ($armPolicyContent -notmatch 'observedKeys\s*&\s*actionKeyMask' -or
        $armPolicyContent -match 'observedKeys\s*==\s*0u' -or
        $armTaskContent -notmatch 'actionKeyMask\s*\|=\s*g_arm_motor_table\[i\]\.key_mask' -or
        $armTaskContent -notmatch 'ArmInputGateSync[\s\S]{0,180}?manualInput\.authoritySeq[\s\S]{0,180}?manualInput\.semanticsSeq') {
        Add-CheckError "${armPolicyRepoPath}: Arm recovery must wait only for action keys declared by the target motor table."
    }

    $servoPolicyRepoPath = "shared\application\services\servo\ServoInputPolicy.h"
    $servoPolicyContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $servoPolicyRepoPath) -Raw -Encoding UTF8
    $servoTaskRepoPath = "shared\application\services\servo\ServoControlTask.c"
    $servoTaskContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $servoTaskRepoPath) -Raw -Encoding UTF8
    if ($servoPolicyContent -notmatch 'ServoInputGateReady\s*\(' -or
        $servoTaskContent -notmatch 'ServoInputGateSync[\s\S]{0,180}?manualInput\.authoritySeq[\s\S]{0,180}?manualInput\.semanticsSeq' -or
        $servoTaskContent -notmatch 'servoOutputAllowed[\s\S]{0,160}?ServoInputGateReady\s*\(' -or
        $servoTaskContent -notmatch 'if\s*\(\s*servoOutputAllowed\s*==\s*0u\s*\)[\s\S]{0,100}?ServoPwmSet\s*\(\s*0u') {
        Add-CheckError "${servoTaskRepoPath}: Servo must keep physical PWM disabled until a post-recovery action-key release frame."
    }

    $chassisPolicyRepoPath = "shared\application\chassis\ChassisInputPolicy.h"
    $chassisPolicyContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $chassisPolicyRepoPath) -Raw -Encoding UTF8
    $chassisBehaviourRepoPath = "shared\application\chassis\ChassisBehaviour.c"
    $chassisBehaviourContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $chassisBehaviourRepoPath) -Raw -Encoding UTF8
    $chassisTaskRepoPath = "shared\application\chassis\ChassisControlTask.c"
    $chassisTaskContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $chassisTaskRepoPath) -Raw -Encoding UTF8
    if ($chassisPolicyContent -notmatch 'authoritySeq[\s\S]{0,80}?semanticsSeq[\s\S]{0,400}?waitRelease' -or
        $chassisBehaviourContent -notmatch 'ChassisInputGateApply[\s\S]{0,260}?fast->authority_seq[\s\S]{0,120}?fast->semantics_seq' -or
        ([regex]::Matches($chassisTaskContent, 'ChassisBehaviourInputGateBlock\s*\(')).Count -lt 3) {
        Add-CheckError "${chassisBehaviourRepoPath}: spin/swing actions must wait for a real post-unlock release after authority or input-semantics changes."
    }

    $gimbalPolicyRepoPath = "shared\application\gimbal\GimbalInputPolicy.h"
    $gimbalPolicyContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $gimbalPolicyRepoPath) -Raw -Encoding UTF8
    $gimbalBehaviourRepoPath = "shared\application\gimbal\GimbalBehaviour.c"
    $gimbalBehaviourContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $gimbalBehaviourRepoPath) -Raw -Encoding UTF8
    if ($gimbalPolicyContent -notmatch 'GimbalInputGateApplyTurn[\s\S]{0,700}?waitTurnRelease' -or
        $gimbalBehaviourContent -notmatch 'GimbalInputGateApplyTurn[\s\S]{0,260}?fast->authority_seq[\s\S]{0,120}?fast->semantics_seq' -or
        $gimbalBehaviourContent -notmatch 'GimbalTurnaroundCancel\s*\(\s*\)' -or
        $gimbalBehaviourContent -notmatch 'GimbalBehaviourInputGateBlock\s*\(') {
        Add-CheckError "${gimbalBehaviourRepoPath}: 180-degree turn must cancel and wait for key release after lock, authority or input-semantics changes."
    }

    $calibrateRepoPath = "shared\application\services\calibration\CalibrateTask.c"
    $calibrateContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $calibrateRepoPath) -Raw -Encoding UTF8
    if ($calibrateContent -notmatch 'last_authority_seq\s*!=\s*manualInput\.authoritySeq[\s\S]{0,120}?last_semantics_seq\s*!=\s*manualInput\.semanticsSeq') {
        Add-CheckError "${calibrateRepoPath}: partial calibration gestures must reset on representative authority and input-semantics changes."
    }

    $auxTelemRepoPath = "shared\application\comm\host\AuxTelem.c"
    $auxTelemContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $auxTelemRepoPath) -Raw -Encoding UTF8
    if (([regex]::Matches($auxTelemContent, 'ManualInputSnapshotRead\s*\(')).Count -ne 1 -or
        $auxTelemContent -match 'RC_data_is_error\s*\(' -or
        $auxTelemContent -notmatch 'manualDataValid\s*=\s*s_aux_telem_manual_input\.dataValid' -or
        $auxTelemContent -notmatch 'control->sw\s*\[\s*INPUT_SW_CHASSIS_MODE\s*\]' -or
        $auxTelemContent -notmatch 'toe_is_error\s*\(\s*DBUS_TOE\s*\)\s*\)\s*mask\s*\|=\s*1u\s*<<\s*0') {
        Add-CheckError "${auxTelemRepoPath}: one telemetry frame must reuse one aggregate snapshot for RC error and mapped chassis switch signals."
    }

    $watchDiagRepoPath = "shared\application\services\diagnostics\WatchDiagCopy.inc"
    $watchDiagContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $watchDiagRepoPath) -Raw -Encoding UTF8
    if ($watchDiagContent -match 'else\s+if\s*\(\s*dbus_error\s*!=\s*0u\s*\|\|\s*manual_online\s*==\s*0u\s*\)') {
        Add-CheckError "${watchDiagRepoPath}: physical DBUS loss may keep its diagnostic bit but cannot mean aggregate manual-input loss."
    }

    $startupRepoPath = "shared\application\services\startup\StartupServiceTask.c"
    $startupContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $startupRepoPath) -Raw -Encoding UTF8
    if ($startupContent -notmatch 'lost_beep_confirm_due\s*\(\s*lifecycle\.manual_online\s*\)' -or
        $startupContent -notmatch 'toe\s*==\s*\(uint8_t\)DBUS_TOE\s*&&\s*manualOnline\s*!=\s*0u') {
        Add-CheckError "${startupRepoPath}: DBUS beep confirmation must share the aggregate-input fallback rule used by the main scan."
    }

    $h7FreertosRepoPath = "boards\DmMc02H7\app\BoardFreertos.c"
    $h7FreertosContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $h7FreertosRepoPath) -Raw -Encoding UTF8
    $heroMDetectRepoPath = "Robotconfig\HERO-M\DetectTask.c"
    $heroMDetectContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $heroMDetectRepoPath) -Raw -Encoding UTF8
    if (([regex]::Matches($h7FreertosContent, 'WatchInit\s*\(')).Count -ne 1 -or
        ([regex]::Matches($h7FreertosContent, 'WatchUpdate\s*\(')).Count -ne 1 -or
        $heroMDetectContent -match 'Watch(?:Init|Update)\s*\(') {
        Add-CheckError "${heroMDetectRepoPath}: H7 Watch must keep StartDefaultTask as its only init/update owner."
    }
    foreach ($controlSummaryRepoPath in @(
            "Robotconfig\HERO-C\DetectTask.c",
            "Robotconfig\HERO-M\DetectTask.c"
        )) {
        $controlSummaryContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $controlSummaryRepoPath) -Raw -Encoding UTF8
        $controlSummaryBody = [regex]::Match(
            $controlSummaryContent,
            'static\s+void\s+sdlog_pack_control_summary\s*\([^)]*\)\s*\{[\s\S]*?(?=\r?\n\})').Value
        if (([regex]::Matches($controlSummaryBody, 'ManualInputSnapshotRead\s*\(')).Count -ne 1 -or
            $controlSummaryBody -match 'g_watch\.rc' -or
            $controlSummaryBody -notmatch 'manual_source\s*=\s*\([^;]*?online' -or
            $controlSummaryBody -notmatch 'manual\.rc\.s\[0\]' -or
            $controlSummaryBody -notmatch 'manual\.rc\.ch\[i\]') {
            Add-CheckError "${controlSummaryRepoPath}: control summary source, switches and axes must come from one aggregate input snapshot."
        }
    }

    $removedInputApiPattern =
        'ManualInputGet(?:CurrentRc|CurrentCopy|ActiveSource)\s*\(|' +
        'get_remote_control_point\s*\(|remote_control_get_active_source\s*\(|' +
        'ControlInput(?:UpdateFromManualInput|GetState|GetCopy|Axis|Switch)\s*\(|' +
        '\binput_(?:update_from_rc|get|get_copy|axis|switch|switch_pos_to_raw|switch_is_pos)\s*\(|' +
        'ImageRemote(?:AutoAimRequested|AuxFireRequested)\s*\('
    foreach ($runtimeInputRoot in @("shared\application", "boards", "Robotconfig")) {
        foreach ($source in (Get-ChildItem -LiteralPath (Join-Path $script:RepoRoot $runtimeInputRoot) -Recurse -File |
                Where-Object { $_.Extension -in @(".c", ".h", ".inc") })) {
            $sourceText = Get-Content -LiteralPath $source.FullName -Raw -Encoding UTF8
            if ($sourceText -match $removedInputApiPattern) {
                Add-CheckError "$(Format-RepoPath $source.FullName): removed parallel input API must not return."
            }
        }
    }
}

function Test-ControlRegistryBoundaries {
    Write-Host "[check] control registry boundaries"

    $repoPath = "shared\application\robot\RobotControlRegistry.h"
    $fullPath = Join-Path $script:RepoRoot $repoPath
    if (-not (Test-Path -LiteralPath $fullPath -PathType Leaf)) {
        Add-CheckError "Missing control registry header: $repoPath"
        return
    }

    $content = Get-Content -LiteralPath $fullPath -Raw
    foreach ($required in @("RobotControlBootstrapProfileDefaults", "RobotControlStartProfileDefaults", "ChassisCtrlDesc", "ShootCtrlDesc")) {
        if ($content -notmatch [regex]::Escape($required)) {
            Add-CheckError "${repoPath}: control registry must expose '$required'."
        }
    }

    foreach ($forbidden in @("ControlMgrSwitchByName", "ControlMgrUpdateDueAll", "ControlMgrUpdateAll")) {
        if ($content -match [regex]::Escape($forbidden)) {
            Add-CheckError "${repoPath}: default controller bootstrap must not use low-rate name lookup or due scheduling via '$forbidden'."
        }
    }
    if ($content -match 'controller\s*==\s*NULL') {
        Add-CheckError "${repoPath}: enabled modules must pass missing descriptors to ControlMgrRegister so registration diagnostics remain visible."
    }

    $controlMgrRepoPath = "shared\application\robot\ControlMgr.c"
    $controlMgrContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $controlMgrRepoPath) -Raw -Encoding UTF8
    foreach ($required in @(
            "reserved_claim_mask",
            "update_in_progress",
            "protected_stop_reason",
            "control_reserved_claim_mask_locked",
            "ControlMgrGetDiag"
        )) {
        if ($controlMgrContent -notmatch [regex]::Escape($required)) {
            Add-CheckError "${controlMgrRepoPath}: hardened lifecycle arbitration must keep '$required'."
        }
    }

    $controlMgrHeaderRepoPath = "shared\application\robot\ControlMgr.h"
    $controlMgrHeaderContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $controlMgrHeaderRepoPath) -Raw -Encoding UTF8
    if ($controlMgrHeaderContent -notmatch 'uint8_t\s+lastRegisterError\s*;\s*uint8_t\s+lastSwitchError\s*;\s*uint8_t\s+reserved\[2\]\s*;') {
        Add-CheckError "${controlMgrHeaderRepoPath}: ControlMgrDiag error fields must use fixed-width storage for ARMCC/clang Watch ABI parity."
    }

    $watchHeaderRepoPath = "shared\application\services\diagnostics\Watch.h"
    $watchHeaderContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $watchHeaderRepoPath) -Raw -Encoding UTF8
    if ($watchHeaderContent -notmatch 'WatchRuntimeDomain\s+domain\[ControlDomainCount\];\s*ControlMgrDiag\s+control_mgr;\s*}\s*WatchRuntime;') {
        Add-CheckError "${watchHeaderRepoPath}: ControlMgrDiag must remain after the WatchRuntime domain array as an append-only ABI field."
    }

    foreach ($testRepoPath in @(
            "tools\TestControlMgr.ps1",
            "tools\tests\ControlMgrRegression.c",
            "tools\tests\ControlMgrAbiRegression.c",
            "tools\tests\ControlMgrTestCritical.h"
        )) {
        if (-not (Test-Path -LiteralPath (Join-Path $script:RepoRoot $testRepoPath) -PathType Leaf)) {
            Add-CheckError "Missing ControlMgr regression file: $testRepoPath"
        }
    }

    $shootCtrlRepoPath = "shared\application\shoot\ShootCtrl.c"
    $shootCtrlContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $shootCtrlRepoPath) -Raw -Encoding UTF8
    foreach ($required in @("ShootCtrlDesc", "ShootCtrlPrepare", "ShootCtrlStep", "ControlMgrUpdateDomain", "ShootCtrlRuntimeStop", "s_shootRuntimeSafe")) {
        if ($shootCtrlContent -notmatch [regex]::Escape($required)) {
            Add-CheckError "${shootCtrlRepoPath}: Shoot lifecycle facade must keep '$required'."
        }
    }

    $chassisCtrlRepoPath = "shared\application\chassis\ChassisCtrl.c"
    $chassisCtrlContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $chassisCtrlRepoPath) -Raw -Encoding UTF8
    foreach ($required in @("ChassisCtrlDesc", "ChassisCtrlPrepare", "ChassisCtrlStep", "ControlMgrUpdateDomain", "ChassisCtrlRuntimeStop", "s_chassisRuntimeSafe", "sensor.imu")) {
        if ($chassisCtrlContent -notmatch [regex]::Escape($required)) {
            Add-CheckError "${chassisCtrlRepoPath}: Chassis lifecycle facade must keep '$required'."
        }
    }
    if ($chassisCtrlContent -match 'state\.ins') {
        Add-CheckError "${chassisCtrlRepoPath}: Chassis descriptor must use the canonical sensor.imu resource name."
    }

    $chassisTaskRepoPath = "shared\application\chassis\ChassisControlTask.c"
    $chassisTaskPath = Join-Path $script:RepoRoot $chassisTaskRepoPath
    $chassisTaskRaw = Get-Content -LiteralPath $chassisTaskPath -Raw -Encoding UTF8
    $chassisTaskContent = Get-SourceContentWithPrivateIncludes -Path $chassisTaskPath
    $chassisPrepareCount = ([regex]::Matches($chassisTaskContent, 'ChassisCtrlPrepare\s*\(')).Count
    $chassisFacadeStepCount = ([regex]::Matches($chassisTaskContent, 'ChassisCtrlStep\s*\(')).Count
    if ($chassisPrepareCount -ne 1 -or $chassisFacadeStepCount -ne 1 -or
        $chassisTaskContent -notmatch 'static\s+ControlResult\s+ChassisTaskRunFrame\s*\(') {
        Add-CheckError "${chassisTaskRepoPath}: Chassis task must prepare once and run exactly one ChassisCtrlStep call point per frame."
    }
    $chassisTaskBody = [regex]::Match(
        $chassisTaskRaw,
        'void\s+ChassisControlTask\s*\([^;]*\)\s*\{[\s\S]*?(?=\r?\n#include\s+"ChassisCoreControl\.inc")'
    ).Value
    if ([string]::IsNullOrWhiteSpace($chassisTaskBody) -or
        ([regex]::Matches($chassisTaskBody, 'ManualInputSnapshotRead\s*\(')).Count -ne 1 -or
        ([regex]::Matches($chassisTaskBody, 'ChassisTaskRunFrame\s*\(')).Count -ne 1 -or
        $chassisTaskBody -notmatch 'ChassisTaskRunFrame\s*\(\s*frameInput\s*,\s*forceSafe\s*\)') {
        Add-CheckError "${chassisTaskRepoPath}: the task loop must read once and pass that exact frameInput into its sole facade call."
    }
    $chassisSourceRoot = Join-Path $script:RepoRoot "shared\application\chassis"
    $chassisInputReadCount = 0
    foreach ($chassisSourceFile in (Get-ChildItem -LiteralPath $chassisSourceRoot -Recurse -File |
            Where-Object { $_.Extension -in @(".c", ".h", ".inc") })) {
        $chassisSourceText = Get-Content -LiteralPath $chassisSourceFile.FullName -Raw -Encoding UTF8
        $sourceReadCount = ([regex]::Matches($chassisSourceText, 'ManualInputSnapshotRead\s*\(')).Count
        $chassisInputReadCount += $sourceReadCount
        if ($sourceReadCount -ne 0 -and $chassisSourceFile.FullName -ne $chassisTaskPath) {
            Add-CheckError "$(Format-RepoPath $chassisSourceFile.FullName): only ChassisControlTask may read the frame-owned manual-input snapshot."
        }
        if ($chassisSourceText -match 'static\s+(?:const\s+)?ManualInputSnapshot\s*\*') {
            Add-CheckError "$(Format-RepoPath $chassisSourceFile.FullName): Chassis must not persist a task-stack manual-input pointer across frames."
        }
    }
    if ($chassisInputReadCount -ne 1) {
        Add-CheckError "shared\application\chassis: aggregate manual input must have exactly one lexical read owner."
    }
    $chassisBehaviourRepoPath = "shared\application\chassis\ChassisBehaviour.c"
    $chassisBehaviourInputContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $chassisBehaviourRepoPath) -Raw -Encoding UTF8
    $chassisInputContent = $chassisTaskContent + "`n" + $chassisBehaviourInputContent
    foreach ($forbiddenPattern in @(
            '\bDBUS_TOE\b',
            'toe_is_error\s*\(',
            'ManualInputGet(CurrentCopy|CurrentRc|ActiveSource)\s*\(',
            'get_remote_control_point\s*\(',
            'remote_control_get_active_source\s*\(',
            'ControlInputGet(Copy|State)\s*\(',
            'ControlInput(Axis|Switch)\s*\(',
            '\binput_(get|axis|switch)\s*\('
        )) {
        if ($chassisInputContent -match $forbiddenPattern) {
            Add-CheckError "${chassisTaskRepoPath}: Chassis control must not use legacy or source-specific input API '$forbiddenPattern'."
        }
    }
    if ($chassisTaskContent -notmatch 'forceSafe\s*=\s*\(uint8_t\)\(manualOffline[\s\S]{0,120}?robot_mode_allow_chassis') {
        Add-CheckError "${chassisTaskRepoPath}: aggregate-input offline and forbidden run modes must use forceSafe without stopping the Chassis domain."
    }
    if ($chassisTaskContent -notmatch 'frameInput\s*==\s*NULL\s*\|\|\s*frameInput->online\s*==\s*0u') {
        Add-CheckError "${chassisTaskRepoPath}: Chassis online state must come from the same aggregate snapshot passed into the frame."
    }
    $chassisRunFrame = [regex]::Match($chassisTaskContent, 'static\s+ControlResult\s+ChassisTaskRunFrame[\s\S]*?(?=/\*\*\s*\r?\n\s*\*\s*@brief)').Value
    if ($chassisRunFrame -notmatch 'ChassisCtrlStep\s*\([\s\S]*?MotorInstSetCurrentBindsBestEffort\s*\(') {
        Add-CheckError "${chassisTaskRepoPath}: ChassisTaskRunFrame must publish the ChassisCtrlOutput after its single facade step."
    }
    $chassisRuntimeStep = [regex]::Match($chassisTaskContent, 'void\s+ChassisRuntimeStep\s*\([\s\S]*?(?=void\s+ChassisRuntimeStop\s*\()').Value
    if ([string]::IsNullOrWhiteSpace($chassisRuntimeStep) -or
        $chassisRuntimeStep -match 'MotorInstSetCurrentBindsBestEffort\s*\(') {
        Add-CheckError "${chassisTaskRepoPath}: normal Chassis Runtime may only fill ChassisCtrlOutput; the task owns the LowCmd write."
    }
    $chassisRuntimeInit = [regex]::Match($chassisTaskContent, 'void\s+ChassisRuntimeInit\s*\([\s\S]*?(?=void\s+ChassisRuntimeSafeStep\s*\()').Value
    if ([string]::IsNullOrWhiteSpace($chassisRuntimeInit) -or
        $chassisRuntimeInit -match 'ChassisSnapshotCapture\s*\(') {
        Add-CheckError "${chassisTaskRepoPath}: Chassis Runtime init must defer the first feedback snapshot to the same-frame update."
    }
    if ($chassisTaskContent -notmatch 'ChassisSnapshotCapture\s*\(\s*snapshot\s*,\s*&g_chassis\s*,\s*manualInput\s*,\s*tickMs\s*,\s*periodMs\s*\)' -or
        $chassisTaskContent -notmatch 'ChassisSdLogAppendBaseSample\s*\(\s*&sample\s*,\s*snapshot\.tick_ms') {
        Add-CheckError "${chassisTaskRepoPath}: Chassis Runtime must carry facade tick/period through snapshot and logging."
    }
    if ($chassisRunFrame -notmatch '\.manualInput\s*=\s*manualInput' -or
        $chassisCtrlContent -notmatch 'ChassisRuntimeSafeStep\s*\(\s*input->manualInput\s*,' -or
        $chassisCtrlContent -notmatch 'ChassisRuntimeStep\s*\(\s*input->manualInput\s*,' -or
        $chassisTaskContent -notmatch 'ChassisRuntimeReadFrame\s*\(\s*&snapshot\s*,\s*manualInput\s*,') {
        Add-CheckError "${chassisTaskRepoPath}: Task, ChassisCtrl, normal/safe Runtime and capture must pass the same input snapshot pointer."
    }
    $chassisCtrlTestRepoPath = "tools\tests\ChassisCtrlRegression.c"
    $chassisCtrlTestPath = Join-Path $script:RepoRoot $chassisCtrlTestRepoPath
    if (-not (Test-Path -LiteralPath $chassisCtrlTestPath -PathType Leaf)) {
        Add-CheckError "Missing Chassis facade regression: $chassisCtrlTestRepoPath"
    }
    else {
        $chassisCtrlTestContent = Get-Content -LiteralPath $chassisCtrlTestPath -Raw -Encoding UTF8
        if ([regex]::Matches($chassisCtrlTestContent, 's_lastManualInput\s*==\s*input\.manualInput').Count -lt 3 -or
            $chassisCtrlTestContent -notmatch 'input\.manualInput\s*=\s*NULL' -or
            $chassisCtrlTestContent -notmatch 's_lastManualInput\s*==\s*NULL' -or
            $chassisCtrlTestContent -notmatch 'ChassisRuntimeSafeStep\s*\(\s*const\s+struct\s+ManualInputSnapshot\s*\*\s*manualInput' -or
            $chassisCtrlTestContent -notmatch 'ChassisRuntimeStep\s*\(\s*const\s+struct\s+ManualInputSnapshot\s*\*\s*manualInput') {
            Add-CheckError "${chassisCtrlTestRepoPath}: regression must prove normal and safe Runtime receive the facade snapshot pointer unchanged."
        }
    }
    $chassisCtrlRunnerRepoPath = "tools\TestChassisCtrl.ps1"
    if (-not (Test-Path -LiteralPath (Join-Path $script:RepoRoot $chassisCtrlRunnerRepoPath) -PathType Leaf)) {
        Add-CheckError "Missing Chassis facade regression runner: $chassisCtrlRunnerRepoPath"
    }

    $gimbalRepoPath = "shared\application\gimbal\GimbalControlTask.c"
    $gimbalPath = Join-Path $script:RepoRoot $gimbalRepoPath
    $gimbalRaw = Get-Content -LiteralPath $gimbalPath -Raw -Encoding UTF8
    $gimbalContent = Get-SourceContentWithPrivateIncludes -Path $gimbalPath
    $shootPrepareCount = ([regex]::Matches($gimbalContent, 'ShootCtrlPrepare\s*\(')).Count
    $shootForceSafeCount = ([regex]::Matches($gimbalContent, 'GimbalRunShootControl\s*\(\s*&snapshot\s*,\s*1u\s*\)')).Count
    $shootNormalCount = ([regex]::Matches($gimbalContent, 'GimbalRunShootControl\s*\(\s*&snapshot\s*,\s*0u\s*\)')).Count
    $shootFacadeStepCount = ([regex]::Matches($gimbalContent, 'ShootCtrlStep\s*\(')).Count
    if ($shootPrepareCount -ne 2 -or
        $shootForceSafeCount -ne 2 -or
        $shootNormalCount -ne 2 -or
        $shootFacadeStepCount -ne 1) {
        Add-CheckError "${gimbalRepoPath}: both gimbal owners must prepare Shoot and run exactly one normal or forced-safe ShootCtrl step per frame."
    }

    $singleGimbalTaskBody = [regex]::Match(
        $gimbalRaw,
        'void\s+GimbalControlTask\s*\([^;]*\)\s*\{[\s\S]*?(?=\r?\nvoid\s+DualYawGimbalControlTask\s*\()'
    ).Value
    $dualGimbalTaskBody = [regex]::Match(
        $gimbalRaw,
        'void\s+DualYawGimbalControlTask\s*\([^;]*\)\s*\{[\s\S]*?(?=\r?\n#include\s+"GimbalCaliHelpers\.inc")'
    ).Value
    foreach ($gimbalTask in @(
            [pscustomobject]@{ Name = "GimbalControlTask"; Body = $singleGimbalTaskBody },
            [pscustomobject]@{ Name = "DualYawGimbalControlTask"; Body = $dualGimbalTaskBody }
        )) {
        if ([string]::IsNullOrWhiteSpace($gimbalTask.Body) -or
            ([regex]::Matches($gimbalTask.Body, 'ManualInputSnapshotRead\s*\(')).Count -ne 1 -or
            ([regex]::Matches($gimbalTask.Body, 'GimbalSnapshotCapture\s*\(\s*&snapshot\s*,\s*&g_gimbal\s*,\s*frame_input\s*\)')).Count -ne 1) {
            Add-CheckError "${gimbalRepoPath}: $($gimbalTask.Name) must read once and pass that exact frame_input into its sole snapshot capture."
        }
    }

    $gimbalSourceRoot = Join-Path $script:RepoRoot "shared\application\gimbal"
    $gimbalInputReadCount = 0
    foreach ($source in (Get-ChildItem -LiteralPath $gimbalSourceRoot -Recurse -File |
            Where-Object { $_.Extension -in @(".c", ".h", ".inc") })) {
        $sourceText = Get-Content -LiteralPath $source.FullName -Raw -Encoding UTF8
        $sourceReadCount = ([regex]::Matches($sourceText, 'ManualInputSnapshotRead\s*\(')).Count
        $gimbalInputReadCount += $sourceReadCount
        if ($sourceReadCount -ne 0 -and $source.FullName -ne $gimbalPath) {
            Add-CheckError "$(Format-RepoPath $source.FullName): only the two Gimbal task loops may read manual-input snapshots."
        }
        if ($sourceText -match 'static\s+(?:const\s+)?ManualInputSnapshot\s*\*') {
            Add-CheckError "$(Format-RepoPath $source.FullName): Gimbal must not persist a task-stack manual-input pointer."
        }
    }
    if ($gimbalInputReadCount -ne 2) {
        Add-CheckError "shared\application\gimbal: aggregate manual input must have exactly two lexical read owners, one in each alternative task."
    }

    $shootSourceRoot = Join-Path $script:RepoRoot "shared\application\shoot"
    $shootSourceTexts = @()
    $shootInputReadCount = 0
    foreach ($source in (Get-ChildItem -LiteralPath $shootSourceRoot -Recurse -File |
            Where-Object { $_.Extension -in @(".c", ".h", ".inc") })) {
        $sourceText = Get-Content -LiteralPath $source.FullName -Raw -Encoding UTF8
        $shootSourceTexts += $sourceText
        $shootInputReadCount += ([regex]::Matches($sourceText, 'ManualInputSnapshotRead\s*\(')).Count
        if ($sourceText -match 'static\s+(?:const\s+)?ManualInputSnapshot\s*\*') {
            Add-CheckError "$(Format-RepoPath $source.FullName): Shoot must not persist a task-stack manual-input pointer."
        }
    }
    if ($shootInputReadCount -ne 0) {
        Add-CheckError "shared\application\shoot: Shoot must reuse the owning Gimbal frame and perform zero aggregate-input reads."
    }
    $gimbalShootInputContent = $gimbalContent + "`n" + ($shootSourceTexts -join "`n")
    foreach ($forbiddenPattern in @(
            '\bDBUS_TOE\b',
            'toe_is_error\s*\(\s*DBUS_TOE',
            'ManualInputGet\w*\s*\(',
            'get_remote_control_point\s*\(',
            'remote_control_get_active_source\s*\(',
            'ControlInputGet(?:Copy|State)\s*\(',
            'ControlInput(?:Axis|Switch)\s*\(',
            '\binput_(?:get|axis|switch)\s*\(',
            'ImageRemote(?:GetState|AutoAimRequested|AuxFireRequested)\s*\('
        )) {
        if ($gimbalShootInputContent -match $forbiddenPattern) {
            Add-CheckError "shared\application\gimbal + shoot: control frames must not use legacy or hidden input API '$forbiddenPattern'."
        }
    }
    if ($gimbalContent -notmatch 'sourceFlags\s*&\s*MANUAL_INPUT_SOURCE_FLAG_AUTO_AIM') {
        Add-CheckError "${gimbalRepoPath}: Gimbal auto aim must consume the frame-owned sourceFlags field."
    }
    $shootInputContent = $shootSourceTexts -join "`n"
    if ($shootInputContent -notmatch 'sourceProtocol\s*!=\s*MANUAL_INPUT_PROTOCOL_IMAGE_VT13' -or
        $shootInputContent -notmatch 'sourceFlags\s*&\s*MANUAL_INPUT_SOURCE_FLAG_AUX_FIRE') {
        Add-CheckError "shared\application\shoot: Shoot must consume frame-owned VT13 protocol and AUX_FIRE metadata."
    }
    if ($gimbalContent -notmatch 'snapshot->manual_input\s*=\s*manual_input' -or
        $gimbalContent -notmatch 'input\.manualInput\s*=\s*\(snapshot\s*!=\s*NULL\)\s*\?\s*snapshot->manual_input\s*:\s*NULL') {
        Add-CheckError "${gimbalRepoPath}: Gimbal capture and Shoot facade must preserve the same frame-owned input pointer."
    }
    $gimbalHeaderContent = Get-Content -LiteralPath (Join-Path $gimbalSourceRoot "GimbalControlTask.h") -Raw -Encoding UTF8
    $shootHeaderContent = Get-Content -LiteralPath (Join-Path $shootSourceRoot "Shoot.h") -Raw -Encoding UTF8
    if ($gimbalHeaderContent -match '\bGimbalRcCtrl\b' -or $shootHeaderContent -match '\bShootRc\b') {
        Add-CheckError "Gimbal/Shoot persistent state must not retain legacy manual-input pointers."
    }

    $shootRuntimePath = Join-Path $shootSourceRoot "Shoot.c"
    $shootRuntimeContent = Get-Content -LiteralPath $shootRuntimePath -Raw -Encoding UTF8
    $shootCtrlPath = Join-Path $shootSourceRoot "ShootCtrl.c"
    $shootCtrlContent = Get-Content -LiteralPath $shootCtrlPath -Raw -Encoding UTF8
    $shootRuntimeInitBody = [regex]::Match(
        $shootRuntimeContent,
        'void\s+ShootRuntimeInit\s*\([^;]*\)\s*\{[\s\S]*?(?=\r?\nvoid\s+ShootRuntimeSafeStep\s*\()'
    ).Value
    $shootRuntimeSafeBody = [regex]::Match(
        $shootRuntimeContent,
        'void\s+ShootRuntimeSafeStep\s*\([^;]*\)\s*\{[\s\S]*?(?=\r?\n/\*\*)'
    ).Value
    $shootForceSafeBody = [regex]::Match(
        $shootCtrlContent,
        'if\s*\(\s*input->forceSafe\s*!=\s*0u\s*\)\s*\{[\s\S]*?return\s+ControlResultOk\s*;\s*\}'
    ).Value
    if ([string]::IsNullOrWhiteSpace($shootRuntimeInitBody) -or
        $shootRuntimeInitBody -match 'ManualInput|ShootFeedbackUpdate\s*\(') {
        Add-CheckError "shared\application\shoot\Shoot.c: Runtime init must defer input and physical feedback capture to the first frame."
    }
    foreach ($requiredCall in @("ShootFaultUpdate", "ShootFaultSyncInhibit", "ShootFeedbackUpdate", "ShootWriteState")) {
        if ([string]::IsNullOrWhiteSpace($shootRuntimeSafeBody) -or
            $shootRuntimeSafeBody -notmatch ([regex]::Escape($requiredCall) + '\s*\(')) {
            Add-CheckError "shared\application\shoot\Shoot.c: SafeStep must keep '$requiredCall' active every forced-safe frame."
        }
    }
    if ([string]::IsNullOrWhiteSpace($shootForceSafeBody) -or
        $shootForceSafeBody -notmatch 'ShootRuntimeSafeStep\s*\(\s*input->manualInput\s*\)' -or
        $shootForceSafeBody -match 'ShootRuntimeStop\s*\(' -or
        $shootCtrlContent -notmatch 'ShootRuntimeStep\s*\(\s*input->manualInput\s*\)') {
        Add-CheckError "shared\application\shoot\ShootCtrl.c: normal and safe updates must pass the same input pointer without stopping the active domain."
    }

    $shootCtrlTestRepoPath = "tools\tests\ShootCtrlRegression.c"
    $shootCtrlTestContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $shootCtrlTestRepoPath) -Raw -Encoding UTF8
    if ($shootCtrlTestContent -notmatch 's_runtimeStepInputs\[0\]\s*==\s*&firstInput' -or
        $shootCtrlTestContent -notmatch 's_runtimeStepInputs\[2\]\s*==\s*NULL' -or
        $shootCtrlTestContent -notmatch 's_runtimeSafeStepInputs\[0\]\s*==\s*&safeInput' -or
        $shootCtrlTestContent -notmatch 's_runtimeStopCount\s*==\s*1u[\s\S]{0,300}?forceSafe') {
        Add-CheckError "${shootCtrlTestRepoPath}: regression must prove normal/safe/NULL pointer identity and that forceSafe does not stop Shoot."
    }
    foreach ($requiredTestPath in @(
            "tools\TestShootCtrl.ps1",
            "tools\TestShootInputPolicy.ps1",
            "tools\tests\ShootInputPolicyRegression.c",
            "shared\application\shoot\ShootInputPolicy.h"
        )) {
        if (-not (Test-Path -LiteralPath (Join-Path $script:RepoRoot $requiredTestPath) -PathType Leaf)) {
            Add-CheckError "Missing Shoot input/lifecycle regression asset: $requiredTestPath"
        }
    }
    foreach ($requiredGateCall in @(
            "ShootInputGateSwitch",
            "ShootInputGateSyncSemantics",
            "ShootInputGateSyncSafeMouse",
            "ShootInputGateApplyFrameMouse"
        )) {
        if ($shootRuntimeContent -notmatch ([regex]::Escape($requiredGateCall) + '\s*\(')) {
            Add-CheckError "shared\application\shoot\Shoot.c: frame input must keep the tested '$requiredGateCall' policy."
        }
    }

    $lifecycleRepoPath = "shared\application\robot\RobotLifecycle.c"
    $lifecyclePath = Join-Path $script:RepoRoot $lifecycleRepoPath
    $lifecycleContent = Get-Content -LiteralPath $lifecyclePath -Raw -Encoding UTF8
    $lifecycleHeaderContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot "shared\application\robot\RobotLifecycle.h") -Raw -Encoding UTF8
    $lifecycleUpdateBody = [regex]::Match(
        $lifecycleContent,
        'void\s+RobotLifecycleUpdate\s*\([^;]*\)\s*\{[\s\S]*?(?=\r?\nRobotLifecycleState\s+RobotLifecycleCurrent\s*\()'
    ).Value
    $lifecycleReaderBodies = [regex]::Match(
        $lifecycleContent,
        'RobotLifecycleState\s+RobotLifecycleCurrent\s*\([^;]*\)[\s\S]*?(?=\r?\nvoid\s+RobotLifecycleEnterFault\s*\()'
    ).Value
    $lifecycleCommitBody = [regex]::Match(
        $lifecycleContent,
        'static\s+void\s+RobotLifecycleCommit\s*\([^;]*\)\s*\{[\s\S]*?(?=\r?\nvoid\s+RobotLifecycleInit\s*\()'
    ).Value
    if (([regex]::Matches($lifecycleContent, 'ManualInputSnapshotRead\s*\(')).Count -ne 1 -or
        ([regex]::Matches($lifecycleUpdateBody, 'ManualInputSnapshotRead\s*\(')).Count -ne 1 -or
        $lifecycleContent -match '\bDBUS_TOE\b|toe_is_error\s*\(|ManualInputGet\w*\s*\(|ControlInput(?:Get|Axis|Switch)\s*\(') {
        Add-CheckError "${lifecycleRepoPath}: lifecycle policy must consume one aggregate snapshot only in its explicit writer update."
    }
    if ([string]::IsNullOrWhiteSpace($lifecycleReaderBodies) -or
        $lifecycleReaderBodies -match 'RobotLifecycleUpdate\s*\(|ManualInput\w*\s*\(|ControlInput(?:Get|Axis|Switch)\s*\(') {
        Add-CheckError "${lifecycleRepoPath}: Current, OutputAllowed and GetSnapshot must remain pure cached readers."
    }
    if ($lifecycleHeaderContent -notmatch 'uint32_t\s+update_tick\s*;' -or
        $lifecycleContent -notmatch 'ROBOT_LIFECYCLE_UPDATE_TIMEOUT_MS' -or
        $lifecycleContent -notmatch 'HAL_GetTick\s*\(\s*\)\s*-\s*out->update_tick[\s\S]{0,200}?ROBOT_LIFECYCLE_REASON_UPDATE_STALE') {
        Add-CheckError "${lifecycleRepoPath}: cached readers must fail closed when the sole lifecycle writer stops updating."
    }
    if ($lifecycleHeaderContent -notmatch 'uint32_t\s+manual_semantics_seq\s*;' -or
        $lifecycleContent -notmatch 'manualInput->semanticsSeq\s*!=\s*0u' -or
        $lifecycleContent -notmatch 'manual_semantics_seq\s*!=\s*manualInput->semanticsSeq' -or
        $lifecycleContent -notmatch 'manual_semantics_seq\s*=\s*manualInput->semanticsSeq' -or
        $lifecycleContent -notmatch 'startup_safe_seen\s*=\s*0u') {
        Add-CheckError "${lifecycleRepoPath}: a new input-semantics generation must revoke the previous startup-safe qualification."
    }
    $liveInputSemanticsPattern = '\bg_config\s*\.\s*manual_input\s*\.\s*semantics\b'
    if ($lifecycleContent -match $liveInputSemanticsPattern) {
        Add-CheckError "${lifecycleRepoPath}: lifecycle must interpret a frame with snapshot.semantics, never live g_config semantics."
    }
    foreach ($domainRepoRoot in @(
            "shared\application\gimbal",
            "shared\application\chassis",
            "shared\application\shoot"
        )) {
        $domainRoot = Join-Path $script:RepoRoot $domainRepoRoot
        foreach ($source in (Get-ChildItem -LiteralPath $domainRoot -Recurse -File |
                Where-Object { $_.Extension -in @(".c", ".h", ".inc") })) {
            $sourceText = Get-Content -LiteralPath $source.FullName -Raw -Encoding UTF8
            if ($sourceText -match $liveInputSemanticsPattern) {
                Add-CheckError "$(Format-RepoPath $source.FullName): control domains must interpret frame input with snapshot.semantics, never live g_config semantics."
            }
        }
    }
    if ([string]::IsNullOrWhiteSpace($lifecycleCommitBody) -or
        $lifecycleCommitBody -notmatch 'g_robot_lifecycle\.fault_latched\s*!=\s*0u[\s\S]{0,300}?next_state\s*=\s*ROBOT_LIFECYCLE_FAULT') {
        Add-CheckError "${lifecycleRepoPath}: commit must recheck a concurrently latched fault before allowing output."
    }
    $lifecycleUpdateOwners = @{}
    foreach ($source in (Get-ChildItem -LiteralPath (Join-Path $script:RepoRoot "shared") -Recurse -File |
            Where-Object { $_.Extension -in @(".c", ".inc") })) {
        $sourceText = Get-Content -LiteralPath $source.FullName -Raw -Encoding UTF8
        $callCount = ([regex]::Matches($sourceText, 'RobotLifecycleUpdate\s*\(')).Count
        if ($callCount -ne 0) {
            $lifecycleUpdateOwners[(Format-RepoPath $source.FullName)] = $callCount
        }
    }
    if ($lifecycleUpdateOwners.Count -ne 2 -or
        $lifecycleUpdateOwners[$lifecycleRepoPath] -ne 1 -or
        $lifecycleUpdateOwners["shared\application\comm\can\CanTxTask.c"] -ne 1) {
        Add-CheckError "RobotLifecycleUpdate must have one definition and one scheduling owner in CanTxTask."
    }
    foreach ($requiredTestPath in @(
            "tools\TestRobotLifecycle.ps1",
            "tools\tests\RobotLifecycleRegression.c"
        )) {
        if (-not (Test-Path -LiteralPath (Join-Path $script:RepoRoot $requiredTestPath) -PathType Leaf)) {
            Add-CheckError "Missing RobotLifecycle aggregate-input regression asset: $requiredTestPath"
        }
    }
    $lifecycleRegressionRepoPath = "tools\tests\RobotLifecycleRegression.c"
    $lifecycleRegressionContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $lifecycleRegressionRepoPath) -Raw -Encoding UTF8
    foreach ($requiredPattern in @(
            'g_config\.input\.gimbalModeInvert\s*=\s*1u',
            'TEST_SEMANTICS_OLD_SEQ',
            'TEST_SEMANTICS_NEW_SEQ',
            'snapshot\.manual_semantics_seq\s*==\s*expectedSeq',
            'TEST_SEMANTICS_NEW_SEQ\s*\)[\s\S]{0,300}?ROBOT_LIFECYCLE_REASON_STARTUP_SAFE_REQUIRED'
        )) {
        if ($lifecycleRegressionContent -notmatch $requiredPattern) {
            Add-CheckError "${lifecycleRegressionRepoPath}: semantics-refresh lifecycle regression is missing '$requiredPattern'."
        }
    }

    $sharedSources = @(Get-ChildItem -Path (Join-Path $script:RepoRoot "shared") -Recurse -File |
        Where-Object { $_.Extension -in @(".c", ".h", ".inc") })
    $runtimeAllowed = @(
        "shared/application/shoot/Shoot.c",
        "shared/application/shoot/ShootCtrl.c",
        "shared/application/shoot/ShootRuntime.h"
    )
    $chassisRuntimeAllowed = @(
        "shared/application/chassis/ChassisControlTask.c",
        "shared/application/chassis/ChassisCtrl.c",
        "shared/application/chassis/ChassisRuntime.h"
    )
    foreach ($source in $sharedSources) {
        $sourceContent = Get-Content -LiteralPath $source.FullName -Raw
        $sourceRepoPath = (Format-RepoPath $source.FullName) -replace '\\', '/'

        if ($sourceContent -match 'ControlMgrUpdateDomain\s*\(\s*ControlDomainShoot' -and
            $sourceRepoPath -ne 'shared/application/shoot/ShootCtrl.c') {
            Add-CheckError "${sourceRepoPath}: Shoot domain may only be updated through ShootCtrlStep."
        }
        if ($sourceContent -match 'ControlMgrUpdateDomain\s*\(\s*ControlDomainChassis' -and
            $sourceRepoPath -ne 'shared/application/chassis/ChassisCtrl.c') {
            Add-CheckError "${sourceRepoPath}: Chassis domain may only be updated through ChassisCtrlStep."
        }
        foreach ($runtimeName in @("ShootRuntimeInit", "ShootRuntimeStep", "ShootRuntimeSafeStep", "ShootRuntimeStop")) {
            if ($sourceContent -match ("\b" + $runtimeName + "\s*\(") -and
                $runtimeAllowed -notcontains $sourceRepoPath) {
                Add-CheckError "${sourceRepoPath}: '$runtimeName' is private to ShootCtrl and Shoot runtime."
            }
        }
        foreach ($legacyName in @("ShootInit", "ShootControlLoop", "ShootStopOutputs", "ShootControlMgrAllows")) {
            if ($sourceContent -match ("\b" + $legacyName + "\s*\(")) {
                Add-CheckError "${sourceRepoPath}: legacy Shoot lifecycle entry '$legacyName' must be removed."
            }
        }
        foreach ($runtimeName in @("ChassisRuntimeInit", "ChassisRuntimeStep", "ChassisRuntimeSafeStep", "ChassisRuntimeStop")) {
            if ($sourceContent -match ("\b" + $runtimeName + "\s*\(") -and
                $chassisRuntimeAllowed -notcontains $sourceRepoPath) {
                Add-CheckError "${sourceRepoPath}: '$runtimeName' is private to ChassisCtrl and Chassis runtime."
            }
        }
        if ($sourceContent -match '\bChassisControlMgrAllows\s*\(') {
            Add-CheckError "${sourceRepoPath}: legacy Chassis lifecycle entry 'ChassisControlMgrAllows' must be removed."
        }
    }

    $projectFiles = @(Get-ChildItem -Path (Join-Path $script:RepoRoot "projects") -Recurse -Filter "*.uvprojx")
    foreach ($projectFile in $projectFiles) {
        $projectContent = Get-Content -LiteralPath $projectFile.FullName -Raw
        if ($projectContent -notmatch '<FileName>ShootCtrl\.c</FileName>') {
            Add-CheckError "$(Format-RepoPath $projectFile.FullName): every target must compile ShootCtrl.c."
        }
        if ($projectContent -notmatch '<FileName>ChassisCtrl\.c</FileName>') {
            Add-CheckError "$(Format-RepoPath $projectFile.FullName): every target must compile ChassisCtrl.c."
        }
    }
}

function Test-FaultGuardBoundaries {
    Write-Host "[check] fault guard boundaries"

    $guardRepoPath = "shared\application\services\diagnostics\RobotFaultGuard.h"
    $guardPath = Join-Path $script:RepoRoot $guardRepoPath
    $guardContent = Get-Content -LiteralPath $guardPath -Raw
    foreach ($required in @("CanTxEmergencyStopNow", "BspCanFaultWaitTxIdle", "NVIC_SystemReset", "RobotFaultResetFromException")) {
        if ($guardContent -notmatch [regex]::Escape($required)) {
            Add-CheckError "${guardRepoPath}: shared fatal path must use '$required'."
        }
    }
    if ($guardContent -notmatch 'RobotFaultEnterSafeStateEx[\s\S]{0,500}?CanTxEmergencyStopNow[\s\S]{0,500}?WatchDiagMarkFatal') {
        Add-CheckError "${guardRepoPath}: fatal path must lock and emit safe output before diagnostic work."
    }
    if ($guardContent -match 'for\s*\(\s*;\s*;\s*\)') {
        Add-CheckError "${guardRepoPath}: fatal path must end in bounded reset, not a permanent loop."
    }

    $canTxRepoPath = "shared\application\comm\can\CanTxTask.c"
    $canTxContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $canTxRepoPath) -Raw
    foreach ($required in @(
            "CanTxEmergencyPrepare",
            "CanTxEmergencyTableValid",
            "CAN_TX_EMERGENCY_MAGIC",
            "s_can_tx_emergency_hash",
            "BspCanFaultLock",
            "BspCanFaultTx"
        )) {
        if ($canTxContent -notmatch [regex]::Escape($required)) {
            Add-CheckError "${canTxRepoPath}: emergency output path must use '$required'."
        }
    }
    if ($canTxContent -notmatch 'mit_count\s*>\s*\(uint8_t\)MotorCount' -or
        $canTxContent -notmatch 'route->bus\s*<\s*1u[\s\S]{0,200}?route->std_id\s*>\s*0x7FFu') {
        Add-CheckError "${canTxRepoPath}: fault consumer must bound the cached route count, bus and standard CAN ID."
    }

    $registryRepoPath = "shared\application\robot\RobotControlRegistry.h"
    $registryContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $registryRepoPath) -Raw
    if ($registryContent -notmatch 'RobotControlBootstrapProfileDefaults[\s\S]{0,1000}?CanTxEmergencyPrepare\s*\(') {
        Add-CheckError "${registryRepoPath}: emergency routes must be prepared before RTOS task creation."
    }

    $lifecycleRepoPath = "shared\application\robot\RobotLifecycle.c"
    $lifecycleContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $lifecycleRepoPath) -Raw
    if ($lifecycleContent -notmatch 'startup_safe_seen' -or
        $lifecycleContent -notmatch 'ROBOT_LIFECYCLE_REASON_STARTUP_SAFE_REQUIRED') {
        Add-CheckError "${lifecycleRepoPath}: every MCU start must see a valid manual safe position before ACTIVE."
    }

    $mainFiles = @(Get-ChildItem -Path (Join-Path $script:RepoRoot "projects") -Recurse -Filter "main.c" |
        Where-Object { $_.FullName -match '\\Core\\Src\\main\.c$' } |
        Sort-Object FullName)

    foreach ($mainFile in $mainFiles) {
        $content = Get-Content -LiteralPath $mainFile.FullName -Raw
        $repoPath = Format-RepoPath $mainFile.FullName
        if ($content -notmatch '#include\s+"RobotFaultGuard\.h"') {
            Add-CheckError "${repoPath}: Error_Handler must include RobotFaultGuard.h."
        }
        if ($content -notmatch 'void\s+Error_Handler\s*\(\s*void\s*\)[\s\S]*?RobotFaultRecordAndHalt\s*\(') {
            Add-CheckError "${repoPath}: Error_Handler must record the fault and enter the shared safe halt path."
        }
    }

    $faultItFiles = @(Get-ChildItem -Path (Join-Path $script:RepoRoot "projects") -Recurse -Filter "stm32*xx_it.c" |
        Sort-Object FullName)
    foreach ($itFile in $faultItFiles) {
        $content = Get-Content -LiteralPath $itFile.FullName -Raw
        $repoPath = Format-RepoPath $itFile.FullName
        foreach ($reason in @(
                "ROBOT_FAULT_REASON_NMI",
                "ROBOT_FAULT_REASON_HARDFAULT",
                "ROBOT_FAULT_REASON_MEMMANAGE",
                "ROBOT_FAULT_REASON_BUSFAULT",
                "ROBOT_FAULT_REASON_USAGEFAULT"
            )) {
            if ($content -notmatch $reason) {
                Add-CheckError "${repoPath}: Cortex fault handler must record $reason."
            }
        }
        if ($content -notmatch 'RobotFaultResetFromException\s*\(') {
            Add-CheckError "${repoPath}: Cortex fault handlers must use the scheduler-independent bounded reset path."
        }
    }

    $freertosFiles = @(Get-ChildItem -Path (Join-Path $script:RepoRoot "projects") -Recurse -Filter "freertos.c" |
        Where-Object { $_.FullName -match '\\Core\\Src\\freertos\.c$' } |
        Sort-Object FullName)
    foreach ($freertosFile in $freertosFiles) {
        $content = Get-Content -LiteralPath $freertosFile.FullName -Raw
        $repoPath = Format-RepoPath $freertosFile.FullName
        if ($content -match 'void\s+vApplication(StackOverflowHook|MallocFailedHook)\s*\(' -and
            $content -notmatch 'RobotFaultEnterSafeStateEx\s*\(') {
            Add-CheckError "${repoPath}: FreeRTOS fatal hooks must enter the shared safe state."
        }
    }
}

function Test-FaultIsolationBoundaries {
    Write-Host "[check] fault isolation boundaries"

    $roots = @("shared", "boards", "projects", "Robotconfig") |
        ForEach-Object { Join-Path $script:RepoRoot $_ }
    $sourceFiles = @(Get-ChildItem -Path $roots -Recurse -File |
        Where-Object {
            $_.Extension -in @(".c", ".h", ".inc") -and
            $_.FullName -notmatch '\\(Drivers|Middlewares|USB_DEVICE)\\'
        })

    $globalEntries = @(
        [pscustomobject]@{
            Token = "CanTxEmergencyStopNow"
            Allowed = @("shared/application/comm/can/CanTxTask.c",
                        "shared/application/comm/can/CanTxTask.h",
                        "shared/application/services/diagnostics/RobotFaultGuard.h")
            AllowedRegex = '/Core/Src/stm32[^/]*xx_it\.c$'
        },
        [pscustomobject]@{
            Token = "LowCmdEnterEmergencyStop"
            Allowed = @("shared/application/robot/LowCmd.c",
                        "shared/application/robot/LowCmd.h",
                        "shared/application/services/diagnostics/RobotFaultGuard.h")
            AllowedRegex = '$a'
        },
        [pscustomobject]@{
            Token = "RobotLifecycleEnterFault"
            Allowed = @("shared/application/robot/RobotLifecycle.c",
                        "shared/application/robot/RobotLifecycle.h",
                        "shared/application/services/diagnostics/RobotFaultGuard.h")
            AllowedRegex = '$a'
        },
        [pscustomobject]@{
            Token = "ControlMgrStopAll"
            Allowed = @("shared/application/robot/ControlMgr.c",
                        "shared/application/robot/ControlMgr.h")
            AllowedRegex = '$a'
        }
    )

    foreach ($entry in $globalEntries) {
        foreach ($file in $sourceFiles) {
            $content = Get-Content -LiteralPath $file.FullName -Raw
            if ($content -notmatch ("\b" + [regex]::Escape($entry.Token) + "\s*\(")) {
                continue
            }

            $repoPath = (Format-RepoPath $file.FullName) -replace '\\', '/'
            if ($entry.Allowed -contains $repoPath -or $repoPath -match $entry.AllowedRegex) {
                continue
            }
            Add-CheckError "${repoPath}: $($entry.Token) is reserved for explicit system-fatal paths; device/domain faults must use FaultMgr isolation."
        }
    }

    $guardRepoPath = "shared\application\services\diagnostics\RobotFaultGuard.h"
    $guardContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $guardRepoPath) -Raw -Encoding UTF8
    if ($guardContent -notmatch 'DEVICE_DOMAIN_FAULTS_USE_FAULT_MGR') {
        Add-CheckError "${guardRepoPath}: document that ordinary device and domain faults cannot enter the global fatal path."
    }

    $faultMgrHeaderRepoPath = "shared\application\robot\FaultMgr.h"
    $faultMgrHeaderContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $faultMgrHeaderRepoPath) -Raw -Encoding UTF8
    foreach ($required in @("#define FAULT_MGR_DEVICE_MAX 8u", "#define FAULT_MGR_DOMAIN_MAX 2u")) {
        if ($faultMgrHeaderContent -notmatch [regex]::Escape($required)) {
            Add-CheckError "${faultMgrHeaderRepoPath}: per-task fault storage must keep bounded default '$required'."
        }
    }

    $faultMgrRepoPath = "shared\application\robot\FaultMgr.c"
    $faultMgrContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $faultMgrRepoPath) -Raw
    foreach ($required in @("FaultMgrSetDeviceFault", "FaultMgrDomainAction", "FaultMgrRecoveryReady")) {
        if ($faultMgrContent -notmatch [regex]::Escape($required)) {
            Add-CheckError "${faultMgrRepoPath}: fault policy core must expose '$required'."
        }
    }
    foreach ($forbidden in @("FreeRTOS.h", "task.h", "cmsis_os.h", "RobotConfig.h", "malloc(")) {
        if ($faultMgrContent -match [regex]::Escape($forbidden)) {
            Add-CheckError "${faultMgrRepoPath}: fault policy core must stay independent from '$forbidden'."
        }
    }

    $motorHealthRepoPath = "shared\application\motors\MotorHealth.c"
    $motorHealthContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $motorHealthRepoPath) -Raw
    foreach ($required in @("rxCount", "lastRxTick", "driveState", "MotorHealthRead")) {
        if ($motorHealthContent -notmatch [regex]::Escape($required)) {
            Add-CheckError "${motorHealthRepoPath}: motor health must use '$required'."
        }
    }
    if ($motorHealthContent -match 'feedback->online') {
        Add-CheckError "${motorHealthRepoPath}: the sticky MotorState.online bit cannot be the health source."
    }
    if ($motorHealthContent -notmatch 'ageMs\s*==\s*UINT32_MAX') {
        Add-CheckError "${motorHealthRepoPath}: tolerate only the one-millisecond sample/read race."
    }
    if ($motorHealthContent -match 'ageMs\s*&\s*0x80000000') {
        Add-CheckError "${motorHealthRepoPath}: do not turn every high-bit age into a healthy future timestamp."
    }

    $lowCmdRepoPath = "shared\application\robot\LowCmd.c"
    $lowCmdContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $lowCmdRepoPath) -Raw -Encoding UTF8
    foreach ($required in @("LowCmdWriterValid", "resolved_writer")) {
        if ($lowCmdContent -notmatch [regex]::Escape($required)) {
            Add-CheckError "${lowCmdRepoPath}: LowCmd authority must keep '$required'."
        }
    }
    if ($lowCmdContent -match 'cmds\s*\[\s*i\s*\]\s*\.\s*writer') {
        Add-CheckError "${lowCmdRepoPath}: MotorCmd payload must not override the writer supplied by the API."
    }

    $armMotionRepoPath = "shared\application\arm\ArmMotion.c"
    $armMotionContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $armMotionRepoPath) -Raw -Encoding UTF8
    foreach ($forbidden in @("CanMitMotorSendCmd", "CanMitMotorSendEnable", "CanMitMotorSendStop", "UnitreeMotorSendActuator", "UnitreeMotorRefresh")) {
        if ($armMotionContent -match ("\b" + [regex]::Escape($forbidden) + "\s*\(")) {
            Add-CheckError "${armMotionRepoPath}: Arm may only publish LowCmd; physical MIT sender '$forbidden' belongs to CanTx."
        }
    }
    foreach ($required in @("LowCmdInhibitManyFrom", "LowCmdReleaseInhibitManyFrom", "ArmFaultSyncInhibit")) {
        if ($armMotionContent -notmatch [regex]::Escape($required)) {
            Add-CheckError "${armMotionRepoPath}: Arm fault isolation must use '$required'."
        }
    }
    $armClearMatch = [regex]::Match(
        $armMotionContent,
        'static\s+void\s+ArmClearMitLowCmd\s*\(uint8_t\s+index\)\s*\{[\s\S]*?(?=\r?\nstatic\s+fp32\s+ArmJ0UnitreeRatioSafe)')
    if (-not $armClearMatch.Success -or $armClearMatch.Value -notmatch 'MotorInstClearId\s*\(') {
        Add-CheckError "${armMotionRepoPath}: ArmClearMitLowCmd must make the axis inactive with MotorInstClearId."
    }
    elseif ($armClearMatch.Value -match 'MotorInstSetSpeedId\s*\(') {
        Add-CheckError "${armMotionRepoPath}: ArmClearMitLowCmd cannot publish active zero speed because that may re-enable MIT."
    }

    $shootRepoPath = "shared\application\shoot\Shoot.c"
    $shootContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $shootRepoPath) -Raw -Encoding UTF8
    foreach ($required in @("ShootFaultSyncInhibit", "LowCmdInhibitManyFrom", "LowCmdReleaseInhibitManyFrom", "GimbalStateReadFresh", "ShootGimbalStateBlocksFire")) {
        if ($shootContent -notmatch [regex]::Escape($required)) {
            Add-CheckError "${shootRepoPath}: shoot fault isolation must use '$required'."
        }
    }

    $axisPolicyRepoPath = "shared\application\motors\MotorAxisFaultPolicy.h"
    $axisPolicyContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $axisPolicyRepoPath) -Raw -Encoding UTF8
    foreach ($required in @("MotorAxisFaultInhibitPlanMake", "releaseMask", "holdZeroMask")) {
        if ($axisPolicyContent -notmatch [regex]::Escape($required)) {
            Add-CheckError "${axisPolicyRepoPath}: shared per-axis recovery policy must keep '$required'."
        }
    }
    foreach ($forbidden in @("FreeRTOS.h", "task.h", "cmsis_os.h", "LowCmd.h")) {
        if ($axisPolicyContent -match [regex]::Escape($forbidden)) {
            Add-CheckError "${axisPolicyRepoPath}: pure per-axis policy cannot depend on '$forbidden'."
        }
    }

    foreach ($faultHelper in @(
            [pscustomobject]@{
                Path = "shared\application\gimbal\GimbalFaultHelpers.inc"
                Sync = "GimbalFaultSyncInhibit"
            },
            [pscustomobject]@{
                Path = "shared\application\chassis\ChassisFaultHelpers.inc"
                Sync = "ChassisFaultSyncInhibit"
            }
        )) {
        $faultContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $faultHelper.Path) -Raw -Encoding UTF8
        foreach ($required in @(
                "FaultMgrSetDeviceFault",
                "LowCmdInhibitManyFrom",
                "LowCmdClearManyFrom",
                "LowCmdReleaseInhibitManyFrom",
                $faultHelper.Sync
            )) {
            if ($faultContent -notmatch [regex]::Escape($required)) {
                Add-CheckError "$($faultHelper.Path): per-axis isolation must keep '$required'."
            }
        }
        if ($faultContent -match 'mask\s*&\s*bit\)\s*==\s*0u\s*\|\|[\s\S]{0,120}?configuredMask') {
            Add-CheckError "$($faultHelper.Path): release collection must not filter old held axes by the current configured mask."
        }
    }

    $gimbalPolicyRepoPath = "shared\application\gimbal\GimbalFaultPolicy.h"
    $gimbalPolicyContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $gimbalPolicyRepoPath) -Raw -Encoding UTF8
    foreach ($required in @("GimbalFaultImuAxisMask", "GimbalFaultAimAxisMask", "ROBOT_RUN_VARIANT_GIMBAL_YAW_ONLY", "ROBOT_RUN_VARIANT_GIMBAL_PITCH_ONLY")) {
        if ($gimbalPolicyContent -notmatch [regex]::Escape($required)) {
            Add-CheckError "${gimbalPolicyRepoPath}: IMU isolation scope must keep '$required'."
        }
    }

    $chassisRepoPath = "shared\application\chassis\ChassisControlTask.c"
    $chassisContent = Get-SourceContentWithPrivateIncludes -Path (Join-Path $script:RepoRoot $chassisRepoPath)
    foreach ($required in @("ChassisFaultUpdate", "ChassisFaultSyncInhibit", "MotorInstSetCurrentBindsBestEffort")) {
        if ($chassisContent -notmatch [regex]::Escape($required)) {
            Add-CheckError "${chassisRepoPath}: classic chassis per-axis isolation must keep '$required'."
        }
    }
    $normalPowerFlowPattern =
        'PID_calc\s*\([\s\S]{0,500}?' +
        'ChassisFaultApplyControl\s*\(\s*ChassisMoveControlLoop\s*\)[\s\S]{0,700}?' +
        'pre_power_current\s*\[\s*i\s*\]\s*=[\s\S]{0,500}?' +
        'ChassisPowerControl\s*\(\s*ChassisMoveControlLoop\s*,\s*ChassisFaultPowerEligibleMask\s*\(\s*\)\s*\)'
    if ($chassisContent -notmatch $normalPowerFlowPattern) {
        Add-CheckError "${chassisRepoPath}: normal chassis flow must clear fault-held axes before pre-power sampling and shared power limiting."
    }
    $rawPowerFlowPattern =
        'if\s*\(\s*ChassisMoveControlLoop->mode\s*==\s*CHASSIS_VECTOR_RAW\s*\)[\s\S]{0,900}?' +
        'give_current\s*=[\s\S]{0,400}?' +
        'ChassisFaultApplyControl\s*\(\s*ChassisMoveControlLoop\s*\)[\s\S]{0,400}?' +
        'pre_power_current\s*\[\s*i\s*\]\s*=[\s\S]{0,300}?return\s*;'
    if ($chassisContent -notmatch $rawPowerFlowPattern) {
        Add-CheckError "${chassisRepoPath}: raw chassis flow must clear fault-held axes before pre-power sampling and return."
    }
    if ($chassisContent -notmatch 'ChassisCurrentCmd\s*\[\s*i\s*\][\s\S]{0,220}?configuredMask') {
        Add-CheckError "${chassisRepoPath}: runtime-disabled chassis axes must be filtered at the final current output."
    }
    foreach ($required in @("LowStateGetMotorMany", "MotorHealthEval", "snapshot->tick_ms")) {
        if ($chassisContent -notmatch [regex]::Escape($required)) {
            Add-CheckError "${chassisRepoPath}: classic chassis frame snapshot must keep '$required'."
        }
    }
    if (([regex]::Matches($chassisContent, 'LowStateGetMotorMany\s*\(')).Count -ne 1) {
        Add-CheckError "${chassisRepoPath}: classic chassis must copy all configured wheel feedback in one LowState batch."
    }
    foreach ($forbidden in @("MotorHealthRead(", "get_chassis_motor_measure_point")) {
        if ($chassisContent -match [regex]::Escape($forbidden)) {
            Add-CheckError "${chassisRepoPath}: classic chassis high-rate path cannot use legacy per-axis source '$forbidden'."
        }
    }
    if ($chassisContent -match '\.\s*ChassisMotorMeasure\b') {
        Add-CheckError "${chassisRepoPath}: classic chassis high-rate path cannot retain the legacy CAN RX measure pointer."
    }
    if (([regex]::Matches($chassisContent, 'GimbalStateReadFresh\s*\(')).Count -ne 1) {
        Add-CheckError "${chassisRepoPath}: classic chassis must capture GimbalState exactly once per frame."
    }
    if ($chassisContent -notmatch 'ChassisGetTurnaroundFrameYaw\s*\(\s*snapshot\s*,') {
        Add-CheckError "${chassisRepoPath}: core turnaround frame must consume the current chassis snapshot."
    }

    $chassisPowerRepoPath = "shared\application\chassis\ChassisPowerControl.c"
    $chassisPowerContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $chassisPowerRepoPath) -Raw -Encoding UTF8
    $demandBody = [regex]::Match(
        $chassisPowerContent,
        'static\s+uint8_t\s+ChassisPowerLimitCurrentsByDemand\s*\([\s\S]*?(?=\r?\nvoid\s+ChassisPowerControlApplySpeedLimit\s*\()').Value
    if ([string]::IsNullOrWhiteSpace($demandBody) -or
        $demandBody -notmatch '\(activeMotorMask\s*&\s*\(1u\s*<<\s*i\)\)\s*==\s*0u\s*\|\|\s*node->can_id\s*==\s*0u') {
        Add-CheckError "${chassisPowerRepoPath}: demand allocation must exclude both masked and unconfigured axes."
    }
    foreach ($required in @("currents[i] = 0.0f", "power_model_currents[i] = 0.0f")) {
        if ($demandBody -notmatch [regex]::Escape($required)) {
            Add-CheckError "${chassisPowerRepoPath}: inactive demand axes must clear '$required'."
        }
    }

    $chassisBehaviourRepoPath = "shared\application\chassis\ChassisBehaviour.c"
    $chassisBehaviourContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $chassisBehaviourRepoPath) -Raw -Encoding UTF8
    if ($chassisBehaviourContent -match 'GimbalStateRead(Fresh)?\s*\(') {
        Add-CheckError "${chassisBehaviourRepoPath}: behaviour helpers must only consume ChassisMove.fast.gimbal."
    }
    foreach ($required in @("fast.gimbal", "ChassisGimbalTurnaroundIsActive(ChassisMoveMode)")) {
        if ($chassisBehaviourContent -notmatch [regex]::Escape($required)) {
            Add-CheckError "${chassisBehaviourRepoPath}: snapshot-only gimbal behaviour must keep '$required'."
        }
    }

    $chassisHeaderRepoPath = "shared\application\chassis\ChassisControlTask.h"
    $chassisHeaderContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $chassisHeaderRepoPath) -Raw -Encoding UTF8
    foreach ($required in @("motor_measure_t measure", "measureValid", "ChassisGimbalSnapshot gimbal")) {
        if ($chassisHeaderContent -notmatch [regex]::Escape($required)) {
            Add-CheckError "${chassisHeaderRepoPath}: persistent frame-owned chassis feedback must keep '$required'."
        }
    }
    if ($chassisHeaderContent -match 'ChassisMotorMeasure') {
        Add-CheckError "${chassisHeaderRepoPath}: ChassisMotor cannot retain a pointer into the CAN RX storage."
    }
    if ($chassisHeaderContent -match '\bChassisRc\s*;' -or
        $chassisHeaderContent -match 'ManualInputSnapshot\s*\*') {
        Add-CheckError "${chassisHeaderRepoPath}: persistent Chassis state must not retain a frame-owned manual-input pointer."
    }
    $runtimeInitBody = [regex]::Match(
        $chassisContent,
        'void\s+ChassisRuntimeInit\s*\([^)]*\)[\s\S]*?(?=void\s+ChassisRuntimeSafeStep\s*\()').Value
    if ($chassisContent -notmatch 'ChassisInit\s*\(\s*&g_chassis\s*\)' -or
        [string]::IsNullOrWhiteSpace($runtimeInitBody) -or
        $runtimeInitBody -match 'ChassisRuntimeSnapshot') {
        Add-CheckError "${chassisRepoPath}: chassis init must not allocate or capture a feedback snapshot before the first same-frame update."
    }

    $chassisTaskFiles = @(
        Get-ChildItem -Path @(
            (Join-Path $script:RepoRoot "projects"),
            (Join-Path $script:RepoRoot "boards")
        ) -Recurse -File |
            Where-Object { $_.Name -in @("freertos.c", "BoardFreertos.c") }
    )
    foreach ($taskFile in $chassisTaskFiles) {
        $taskContent = Get-Content -LiteralPath $taskFile.FullName -Raw
        $stackMatch = [regex]::Match(
            $taskContent,
            '(?:APP_STATIC_THREAD|APP_THREAD_ATTR)\s*\(\s*chassisControlTask\s*,[^\r\n]*?,\s*(\d+)\s*\)')
        if ($stackMatch.Success -and [int]$stackMatch.Groups[1].Value -lt 768) {
            Add-CheckError "$(Format-RepoPath $taskFile.FullName): classic chassis task needs at least 768 stack words."
        }
    }

    $taskStackMinimums = @(
        [pscustomobject]@{ Name = "hostLinkTask"; Minimum = 512; Label = "host link" },
        [pscustomobject]@{ Name = "startupServiceTask"; Minimum = 768; Label = "startup service" },
        [pscustomobject]@{ Name = "healthMonitorTask"; Minimum = 384; Label = "health monitor" },
        [pscustomobject]@{ Name = "servoControlTask"; Minimum = 192; Label = "servo control" }
    )
    foreach ($taskFile in $chassisTaskFiles) {
        $taskContent = Get-Content -LiteralPath $taskFile.FullName -Raw
        foreach ($stackRule in $taskStackMinimums) {
            $stackMatch = [regex]::Match(
                $taskContent,
                '(?:APP_STATIC_THREAD|APP_THREAD_ATTR)\s*\(\s*' +
                    [regex]::Escape($stackRule.Name) +
                    '\s*,[^\r\n]*?,\s*(\d+)\s*\)',
                [System.Text.RegularExpressions.RegexOptions]::IgnoreCase)
            if ($stackMatch.Success -and [int]$stackMatch.Groups[1].Value -lt $stackRule.Minimum) {
                Add-CheckError "$(Format-RepoPath $taskFile.FullName): $($stackRule.Label) task needs at least $($stackRule.Minimum) stack words."
            }
        }
    }

    $robotModuleRepoPath = "shared\application\robot\RobotModule.h"
    $robotModuleContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $robotModuleRepoPath) -Raw -Encoding UTF8
    if ($robotModuleContent -notmatch 'ROBOT_TASK_MODULE_CLASSIC_CHASSIS[\s\S]{0,260}?ROBOT_PROFILE_CHASSIS_CONTROL_BUDGET_US,\s*768u,') {
        Add-CheckError "${robotModuleRepoPath}: classic chassis descriptor must publish the 768-word minimum stack."
    }
    foreach ($moduleStackRule in @(
            [pscustomobject]@{ Module = "ROBOT_TASK_MODULE_HEALTH_MONITOR"; Minimum = 384 },
            [pscustomobject]@{ Module = "ROBOT_TASK_MODULE_HOST_LINK"; Minimum = 512 },
            [pscustomobject]@{ Module = "ROBOT_TASK_MODULE_STARTUP_SERVICE"; Minimum = 768 },
            [pscustomobject]@{ Module = "ROBOT_TASK_MODULE_SERVO"; Minimum = 192 }
        )) {
        if ($robotModuleContent -notmatch ($moduleStackRule.Module + '[\s\S]{0,260}?' + $moduleStackRule.Minimum + 'u,')) {
            Add-CheckError "${robotModuleRepoPath}: $($moduleStackRule.Module) descriptor must publish the $($moduleStackRule.Minimum)-word minimum stack."
        }
    }
    foreach ($source in $sourceFiles) {
        $legacyContent = Get-Content -LiteralPath $source.FullName -Raw
        if ($legacyContent -match '\bChassisMotorMeasure\b') {
            Add-CheckError "$(Format-RepoPath $source.FullName): legacy chassis CAN RX measure pointer must not return."
        }
    }

    $motorStateHeaderRepoPath = "shared\application\robot\LowCmd.h"
    $motorStateHeaderContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $motorStateHeaderRepoPath) -Raw -Encoding UTF8
    if ($motorStateHeaderContent -notmatch '\buint16_t\s+lastEcd\s*;') {
        Add-CheckError "${motorStateHeaderRepoPath}: MotorState must preserve the previous received encoder value."
    }
    $ecdPolicyRepoPath = "shared\application\motors\MotorFeedbackEcdPolicy.h"
    $ecdPolicyContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $ecdPolicyRepoPath) -Raw -Encoding UTF8
    foreach ($required in @("MotorFeedbackRxCountNext", "MotorFeedbackEcdResolve", "rxCount != previous->rxCount", "previous->lastEcd")) {
        if ($ecdPolicyContent -notmatch [regex]::Escape($required)) {
            Add-CheckError "${ecdPolicyRepoPath}: repeated RS485 refreshes must preserve '$required'."
        }
    }
    $canRxRepoPath = "shared\application\comm\can\CanReceive.c"
    $canRxContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $canRxRepoPath) -Raw -Encoding UTF8
    if (([regex]::Matches($canRxContent, 'MotorFeedbackRxCountNext\s*\(')).Count -lt 2) {
        Add-CheckError "${canRxRepoPath}: CAN feedback rxCount must skip the reserved zero value after wraparound."
    }
    foreach ($driverRepoPath in @(
            "shared\application\motors\UnitreeMotorDriver.c",
            "shared\application\motors\N6014bMotorDriver.c"
        )) {
        $driverContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $driverRepoPath) -Raw -Encoding UTF8
        if ($driverContent -notmatch 'MotorFeedbackEcdResolve\s*\(') {
            Add-CheckError "${driverRepoPath}: RS485 feedback must advance lastEcd only on a new receive sample."
        }
    }

    $gimbalRepoPath = "shared\application\gimbal\GimbalControlTask.c"
    $gimbalContent = Get-SourceContentWithPrivateIncludes -Path (Join-Path $script:RepoRoot $gimbalRepoPath)
    foreach ($required in @("GimbalFaultUpdate", "GimbalFaultSyncInhibit", "MotorInstSetCurrentBindsBestEffort")) {
        if ($gimbalContent -notmatch [regex]::Escape($required)) {
            Add-CheckError "${gimbalRepoPath}: gimbal per-axis isolation must keep '$required'."
        }
    }

    $routeRepoPath = "shared\application\comm\can\CanCommandTxRouteHelpers.inc"
    $routeContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $routeRepoPath) -Raw -Encoding UTF8
    foreach ($required in @("CanTxApplyInhibitGate", "CanTxCachedCmdAuthorized", "CanTxRs485DriverOwnsApplied", "CanTxMergeDriverAppliedFlags")) {
        if ($routeContent -notmatch [regex]::Escape($required)) {
            Add-CheckError "${routeRepoPath}: generic CanTx safety closure must keep '$required'."
        }
    }

    $emitRepoPath = "shared\application\comm\can\CanCommandTxEmitHelpers.inc"
    $emitContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $emitRepoPath) -Raw -Encoding UTF8
    if ($emitContent -notmatch 'taskENTER_CRITICAL\s*\(\s*\)[\s\S]*?CanTxRecheckRmFrame\s*\([\s\S]*?CAN_cmd_rm_group\s*\([\s\S]*?taskEXIT_CRITICAL\s*\(\s*\)') {
        Add-CheckError "${emitRepoPath}: RM final authorization and non-blocking enqueue must share the LowCmd task critical section."
    }
    foreach ($required in @("CanTxUpdateApplied", "CanTxLogMotorCmd")) {
        if ($emitContent -notmatch [regex]::Escape($required)) {
            Add-CheckError "${emitRepoPath}: an RM authority rejection must also correct '$required'."
        }
    }

    $unitreePolicyRepoPath = "shared\application\motors\UnitreeMotorPolicy.h"
    $unitreePolicyContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $unitreePolicyRepoPath) -Raw -Encoding UTF8
    foreach ($required in @("UNITREE_MOTOR_DEFAULT_RX_TIMEOUT_MS", "UnitreeMotorCmdSnapshotAllowed", "UnitreeMotorBrakeRequired", "UnitreeMotorMapAppliedOutput", "UnitreeMotorTxDue")) {
        if ($unitreePolicyContent -notmatch [regex]::Escape($required)) {
            Add-CheckError "${unitreePolicyRepoPath}: Unitree single-sender policy must keep '$required'."
        }
    }

    $mitRepoPath = "shared\application\comm\can\CanCommandTxMitHelpers.inc"
    $mitContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $mitRepoPath) -Raw -Encoding UTF8
    if ($mitContent -notmatch 'static\s+uint8_t\s+CanTxMitSendEnableAuthorized[\s\S]*?taskENTER_CRITICAL\s*\(\s*\)[\s\S]{0,260}?CanTxMitCommandAuthorizedNow[\s\S]{0,180}?CanMitMotorSendEnable\s*\([\s\S]{0,120}?taskEXIT_CRITICAL\s*\(\s*\)') {
        Add-CheckError "${mitRepoPath}: MIT Enable final authorization and CAN enqueue must share one short critical section."
    }
    if ($mitContent -notmatch 'static\s+uint8_t\s+CanTxMitSendCmdAuthorized[\s\S]*?taskENTER_CRITICAL\s*\(\s*\)[\s\S]{0,260}?CanTxMitCommandAuthorizedNow[\s\S]{0,220}?CanMitMotorSendCmd\s*\([\s\S]{0,120}?taskEXIT_CRITICAL\s*\(\s*\)' -or
        ([regex]::Matches($mitContent, 'CanTxMitSendCmdAuthorized\s*\(')).Count -lt 3) {
        Add-CheckError "${mitRepoPath}: every MIT command branch must use the linearized final-authority sender."
    }

    $bestEffortRepoPath = "shared\application\motors\MotorInstBestEffort.h"
    $bestEffortContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $bestEffortRepoPath) -Raw -Encoding UTF8
    foreach ($required in @("LowCmdSetMotorMany", "LowCmdSetMotor", "LowCmdSetCurrentMany")) {
        if ($bestEffortContent -notmatch ([regex]::Escape($required) + '\s*\(')) {
            Add-CheckError "${bestEffortRepoPath}: best-effort batch fallback must use '$required'."
        }
    }
    if (([regex]::Matches($bestEffortContent, 'LowCmdSetCurrentMany\s*\(')).Count -lt 2) {
        Add-CheckError "${bestEffortRepoPath}: current fallback must retry LowCmdSetCurrentMany one axis at a time after the normal batch is rejected."
    }

    $watchRuntimeRepoPath = "shared\application\services\diagnostics\WatchRuntimeCopy.inc"
    $watchRuntimeContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $watchRuntimeRepoPath) -Raw -Encoding UTF8
    foreach ($required in @("lowcmd_inhibit_acquire_count", "lowcmd_inhibit_release_count", "lowcmd_inhibit_mask")) {
        if ($watchRuntimeContent -notmatch [regex]::Escape($required)) {
            Add-CheckError "${watchRuntimeRepoPath}: common Watch runtime must copy '$required'."
        }
    }

    foreach ($project in $script:projects) {
        $projectContent = Get-Content -LiteralPath $project.UvprojxPath -Raw
        foreach ($sourceName in @("FaultMgr.c", "MotorHealth.c")) {
            if ($projectContent -notmatch ("<FileName>" + [regex]::Escape($sourceName) + "</FileName>")) {
                Add-CheckError "$($project.Name): Keil project must compile shared fault source $sourceName."
            }
        }
    }

}

function Test-ControlCoreBoundaries {
    Write-Host "[check] control core boundaries"

    $coreFiles = @(
        "shared\application\robot\ControlCore.h",
        "shared\application\arm\ArmCore.h",
        "shared\application\chassis\ChassisCore.h",
        "shared\application\gimbal\GimbalCore.h",
        "shared\application\wheelleg\WheelLegCore.h"
    )
    $forbiddenPatterns = @(
        'FreeRTOS\.h',
        'cmsis_os\.h',
        'task\.h',
        'CanReceive\.h',
        'MotorInst\.h',
        'InsTask\.h',
        'SdLog\.h',
        'Watch\.h',
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
            Source = "shared\application\arm\ArmMotion.c"
            Include = '#include "ArmCore.h"'
            Step = 'ArmCoreStepManual'
        },
        [pscustomobject]@{
            Source = "shared\application\chassis\ChassisControlTask.c"
            Include = '#include "ChassisCore.h"'
            Step = 'ChassisCoreStepVelocity'
        },
        [pscustomobject]@{
            Source = "shared\application\gimbal\GimbalControlTask.c"
            Include = '#include "GimbalCore.h"'
            Step = 'GimbalCoreStepAxisBase'
        },
        [pscustomobject]@{
            Source = "shared\application\wheelleg\WheelLegMitTask.c"
            Include = '#include "WheelLegCore.h"'
            Step = @(
                'WheelLegCoreCalcKinematics',
                'WheelLegCoreSetWheelTorques',
                'WheelLegCoreLqrWheelOutput',
                'WheelLegCoreTargetSmoothUpdateXy',
                'WheelLegCoreObserverUpdate',
                'WheelLegCorePidCalc'
            )
        }
    )

    foreach ($adapter in $adapterIncludes) {
        $fullPath = Join-Path $script:RepoRoot $adapter.Source
        if (-not (Test-Path -LiteralPath $fullPath -PathType Leaf)) {
            Add-CheckError "Missing control adapter source: $($adapter.Source)"
            continue
        }

        $content = Get-SourceContentWithPrivateIncludes -Path $fullPath
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

    $schemaPath = Join-Path $script:RepoRoot "shared\application\robot\RobotConfigSchema.h"
    $devicePath = Join-Path $script:RepoRoot "shared\application\robot\RobotDeviceConfig.h"
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
            "link.AuxTelem",
            "service.sdlog"
        )) {
        if ($schemaContent -notmatch [regex]::Escape($deviceName)) {
            Add-CheckError "$(Format-RepoPath $schemaPath): default device table is missing '$deviceName'."
        }
    }
}

function Test-SharedConfigTypes {
    Write-Host "[check] shared config types"

    $typesPath = Join-Path $script:RepoRoot "shared\application\robot\RobotConfigTypes.h"
    if (-not (Test-Path -LiteralPath $typesPath -PathType Leaf)) {
        Add-CheckError "Missing shared config types header: $(Format-RepoPath $typesPath)"
        return
    }

    $content = Get-Content -LiteralPath $typesPath -Raw
    if ($content -match '\bARBATOS_TARGET_NAME\b|\bROBOT_PROFILE_KIND\s+ROBOT_PROFILE_KIND_|\bROBOT_BOARD_KIND\s+ROBOT_BOARD_KIND_') {
        Add-CheckError "$(Format-RepoPath $typesPath): shared config types must not contain target identity macros."
    }
    if ($content -notmatch 'typedef\s+struct[\s\S]*?\}\s*Config\s*;') {
        Add-CheckError "$(Format-RepoPath $typesPath): cannot find shared Config definition."
    }
    if ($content -notmatch '#ifndef\s+MOTOR_ARM_JOINT_COUNT[\s\S]*?#define\s+MOTOR_ARM_JOINT_COUNT\s+6u[\s\S]*?#endif') {
        Add-CheckError "$(Format-RepoPath $typesPath): MOTOR_ARM_JOINT_COUNT must be a target-overridable default."
    }
    if ($content -notmatch '#include\s+"RobotConfigSchema\.h"') {
        Add-CheckError "$(Format-RepoPath $typesPath): shared config types must include RobotConfigSchema.h."
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
Test-ConfigParamGovernance
Test-TaskModuleNames $projects
Test-RobotModuleDescriptors
Test-ProfileIdentity $projects
Test-ProfileProductRules $projects
Test-RtProfilerDescriptors
Test-PythonTools
Test-SimulationTools
Test-BuildManifestTools $projects.Count
Test-StaleText
Test-IncludeFilenameCase
Test-ProjectOwnedPathNames
Test-HighRateApiBoundaries
Test-CanRxAndStackSamplingBoundaries
Test-CanTxDeviceConfigBoundaries
Test-ManualInputSnapshotBoundaries
Test-ControlRegistryBoundaries
Test-FaultGuardBoundaries
Test-FaultIsolationBoundaries
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
