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
        if ($normPath -eq 'tools/tests/stubs/cmsis_compiler.h') {
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
    foreach ($required in @("RobotControlBootstrapProfileDefaults", "RobotControlStartProfileDefaults", "ShootCtrlDesc")) {
        if ($content -notmatch [regex]::Escape($required)) {
            Add-CheckError "${repoPath}: control registry must expose '$required'."
        }
    }

    foreach ($forbidden in @("ControlMgrSwitchByName", "ControlMgrUpdateDueAll", "ControlMgrUpdateAll")) {
        if ($content -match [regex]::Escape($forbidden)) {
            Add-CheckError "${repoPath}: default controller bootstrap must not use low-rate name lookup or due scheduling via '$forbidden'."
        }
    }

    $shootCtrlRepoPath = "shared\application\shoot\ShootCtrl.c"
    $shootCtrlContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $shootCtrlRepoPath) -Raw -Encoding UTF8
    foreach ($required in @("ShootCtrlDesc", "ShootCtrlPrepare", "ShootCtrlStep", "ControlMgrUpdateDomain", "ShootCtrlRuntimeStop", "s_shootRuntimeSafe")) {
        if ($shootCtrlContent -notmatch [regex]::Escape($required)) {
            Add-CheckError "${shootCtrlRepoPath}: Shoot lifecycle facade must keep '$required'."
        }
    }

    $gimbalRepoPath = "shared\application\gimbal\GimbalControlTask.c"
    $gimbalContent = Get-SourceContentWithPrivateIncludes -Path (Join-Path $script:RepoRoot $gimbalRepoPath)
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

    $sharedSources = @(Get-ChildItem -Path (Join-Path $script:RepoRoot "shared") -Recurse -File |
        Where-Object { $_.Extension -in @(".c", ".h", ".inc") })
    $runtimeAllowed = @(
        "shared/application/shoot/Shoot.c",
        "shared/application/shoot/ShootCtrl.c",
        "shared/application/shoot/ShootRuntime.h"
    )
    foreach ($source in $sharedSources) {
        $sourceContent = Get-Content -LiteralPath $source.FullName -Raw
        $sourceRepoPath = (Format-RepoPath $source.FullName) -replace '\\', '/'

        if ($sourceContent -match 'ControlMgrUpdateDomain\s*\(\s*ControlDomainShoot' -and
            $sourceRepoPath -ne 'shared/application/shoot/ShootCtrl.c') {
            Add-CheckError "${sourceRepoPath}: Shoot domain may only be updated through ShootCtrlStep."
        }
        foreach ($runtimeName in @("ShootRuntimeInit", "ShootRuntimeStep", "ShootRuntimeStop")) {
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
    }

    $projectFiles = @(Get-ChildItem -Path (Join-Path $script:RepoRoot "projects") -Recurse -Filter "*.uvprojx")
    foreach ($projectFile in $projectFiles) {
        $projectContent = Get-Content -LiteralPath $projectFile.FullName -Raw
        if ($projectContent -notmatch '<FileName>ShootCtrl\.c</FileName>') {
            Add-CheckError "$(Format-RepoPath $projectFile.FullName): every target must compile ShootCtrl.c."
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
    $guardContent = Get-Content -LiteralPath (Join-Path $script:RepoRoot $guardRepoPath) -Raw
    if ($guardContent -notmatch '普通设备掉线[\s\S]{0,160}?FaultMgr') {
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
    if ($chassisContent -notmatch 'ChassisCurrentCmd\s*\[\s*i\s*\][\s\S]{0,220}?configuredMask') {
        Add-CheckError "${chassisRepoPath}: runtime-disabled chassis axes must be filtered at the final current output."
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
    if ($mitContent -notmatch 'CanTxMitCommandAuthorizedNow[\s\S]{0,260}CanMitMotorSendEnable\s*\(') {
        Add-CheckError "${mitRepoPath}: MIT Enable must re-read and authorize the latest LowCmd immediately before send."
    }
    if (([regex]::Matches($mitContent, 'CanTxMitCommandAuthorizedNow[\s\S]{0,260}CanMitMotorSendCmd\s*\(')).Count -lt 2) {
        Add-CheckError "${mitRepoPath}: every MIT command send branch must re-read and authorize the latest LowCmd."
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
