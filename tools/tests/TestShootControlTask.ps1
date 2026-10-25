param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$Zig = Get-Command zig -ErrorAction Stop
$BuildDir = Join-Path $RepoRoot 'local\build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'shoot-task-regression.exe'
$Args = @('cc', '-std=c11', '-Wall', '-Wextra', '-Werror')
foreach ($Dir in @('tools/tests/shoot-task-stubs', 'shared/application/shoot', 'shared/application/robot')) {
    $Args += '-I' + (Join-Path $RepoRoot $Dir)
}
$Args += @((Join-Path $RepoRoot 'tools/tests/ShootControlTaskRegression.c'),
           (Join-Path $RepoRoot 'shared/application/shoot/ShootControlTask.c'), '-o', $Output)
& $Zig.Source @Args
if ($LASTEXITCODE -ne 0) { throw "发射任务主机回归编译失败：$LASTEXITCODE" }
& $Output
if ($LASTEXITCODE -ne 0) { throw "发射任务主机回归失败：$LASTEXITCODE" }
