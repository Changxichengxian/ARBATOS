[CmdletBinding()]
param(
    [ValidateSet('Open', 'Build')]
    [string]$Action = 'Open',

    [string]$Project,

    [string]$ClionPath = $env:CLION_EXE,

    [switch]$Check,

    [switch]$Pause
)

$ErrorActionPreference = 'Stop'
$repoPath = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$projectPath = Join-Path $repoPath 'projects'

function FindClion {
    if ($ClionPath) {
        if (-not (Test-Path -LiteralPath $ClionPath -PathType Leaf)) {
            throw "指定的 CLion 程序不存在：$ClionPath"
        }
        return [IO.Path]::GetFullPath($ClionPath)
    }
    foreach ($name in @('clion64.exe', 'clion.exe')) {
        $command = Get-Command $name -CommandType Application -ErrorAction SilentlyContinue
        if ($command) { return $command.Source }
    }
    # 独立安装器会记录安装位置，升级版本后不必修改根目录的入口。
    foreach ($keyPath in @('HKCU:\SOFTWARE\JetBrains\CLion', 'HKLM:\SOFTWARE\JetBrains\CLion', 'HKLM:\SOFTWARE\WOW6432Node\JetBrains\CLion')) {
        if (-not (Test-Path -LiteralPath $keyPath)) { continue }
        $versions = @(Get-ChildItem -LiteralPath $keyPath | Sort-Object PSChildName -Descending)
        foreach ($version in $versions) {
            $installPath = $version.GetValue('')
            if (-not $installPath) { continue }
            $launcherPath = Join-Path $installPath 'bin\clion64.exe'
            if (Test-Path -LiteralPath $launcherPath -PathType Leaf) { return $launcherPath }
        }
    }
    throw '找不到 CLion。请将 clion64.exe 所在目录加入 PATH，或用 CLION_EXE 环境变量指定完整程序路径。'
}

function SelectProject {
    $generator = Join-Path $repoPath 'tools\config\RobotConfigGen.py'
    if (-not (Test-Path -LiteralPath $generator -PathType Leaf)) {
        throw "找不到车型配置生成器：$generator"
    }
    $pythonPath = Join-Path $repoPath 'local\cache\zephyrproject\.venv\Scripts\python.exe'
    if (-not (Test-Path -LiteralPath $pythonPath -PathType Leaf)) {
        $python = Get-Command python -ErrorAction SilentlyContinue
        if (-not $python) { throw '找不到 Python，无法读取 Robotconfig 车型列表。' }
        $pythonPath = $python.Source
    }
    $targets = @((& $pythonPath $generator list --json) | ConvertFrom-Json)
    if ($LASTEXITCODE -ne 0 -or $targets.Count -eq 0) { throw '读取 Robotconfig 车型列表失败。' }
    if ($Project) { return $Project }
    if ($Check) { return $targets[0].name }
    Write-Host '选择要编译的车型：'
    for ($i = 0; $i -lt $targets.Count; $i++) {
        Write-Host ('  {0}  {1}{2}' -f ($i + 1), $targets[$i].name, $(if ($i -eq 0) { '（默认）' } else { '' }))
    }
    Write-Host '  Q  退出'
    while ($true) {
        $choice = (Read-Host ('输入编号，直接回车选择 ' + $targets[0].name)).Trim()
        if ($choice -eq '') { return $targets[0].name }
        if ($choice -ieq 'q') { return $null }
        $number = 0
        if ([int]::TryParse($choice, [ref]$number) -and $number -ge 1 -and $number -le $targets.Count) {
            return $targets[$number - 1].name
        }
        Write-Host '请输入列表中的编号或 Q。'
    }
}

$resultCode = 0
try {
    if (-not (Test-Path -LiteralPath (Join-Path $projectPath 'CMakeLists.txt') -PathType Leaf)) {
        throw "找不到工程入口：$projectPath"
    }
    if ($Action -eq 'Open') {
        $launcherPath = FindClion
        Write-Host "CLion：$launcherPath"
        Write-Host "工程：$projectPath"
        Write-Host '所有车型共用这个工程；在 CLion 的 CMake 配置中选择对应车型。'
        if (-not $Check) {
            Start-Process -FilePath $launcherPath -ArgumentList ('"{0}"' -f $projectPath) -WorkingDirectory $repoPath
        }
    } else {
        $selectedProject = SelectProject
        if ($selectedProject) {
            Write-Host "编译车型：$selectedProject；并行数：2"
            Write-Host ('产物目录：' + (Join-Path $repoPath ('local\build\' + $selectedProject.ToLowerInvariant() + '\zephyr')))
            if (-not $Check) {
                # 单独的 PowerShell 进程保留 build.ps1 的退出码和环境边界。
                $shellPath = Join-Path $PSHOME 'pwsh.exe'
                & $shellPath -NoProfile -ExecutionPolicy Bypass -File (Join-Path $repoPath 'tools\build.ps1') -Action build -Project $selectedProject -Jobs 2
                $resultCode = $LASTEXITCODE
                if ($resultCode -eq 0) { Write-Host '编译完成。' }
                else { Write-Host "编译失败，退出码 $resultCode。" }
            }
        }
    }
} catch {
    Write-Host $_.Exception.Message -ForegroundColor Red
    $resultCode = 1
}
if ($Pause -and -not $Check) { [void](Read-Host '按回车关闭窗口') }
exit $resultCode
