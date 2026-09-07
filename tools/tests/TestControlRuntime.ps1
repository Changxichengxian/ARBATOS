param()

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$Zig = Get-Command zig -ErrorAction SilentlyContinue
$BuildDir = Join-Path $RepoRoot 'local\build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$Output = Join-Path $BuildDir 'control-runtime-regression.exe'
$CriticalHeader = Join-Path $RepoRoot 'tools\tests\ControlMgrTestCritical.h'
$PlatformHeader = Join-Path $RepoRoot 'tools\tests\ControlRuntimeTestPlatform.h'
$Sources = @(
    (Join-Path $RepoRoot 'tools\tests\ControlRuntimeRegression.c'),
    (Join-Path $RepoRoot 'shared\application\robot\ControlRuntime.c'),
    (Join-Path $RepoRoot 'shared\application\robot\ControlMgr.c')
)
$Args = @(
    'cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
    '-DCONTROL_MANAGER_TEST=1',
    '-include', $CriticalHeader,
    '-include', $PlatformHeader,
    ('-I' + (Join-Path $RepoRoot 'tools\tests')),
    ('-I' + (Join-Path $RepoRoot 'shared\application\robot')),
    ('-I' + (Join-Path $RepoRoot 'shared\application\input')),
    ('-I' + (Join-Path $RepoRoot 'shared\application\motors')),
    ('-I' + (Join-Path $RepoRoot 'shared\application\services\diagnostics')),
    ('-I' + (Join-Path $RepoRoot 'shared\hal')),
    ('-I' + (Join-Path $RepoRoot 'shared\components\support'))
)
$Args += $Sources
$Args += @('-o', $Output)

if ($null -ne $Zig) {
    & $Zig.Source @Args
    if ($LASTEXITCODE -ne 0) {
        throw "通用控制器回归编译失败，退出码 $LASTEXITCODE"
    }
}
else {
    $VsWhere = 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
    if (-not (Test-Path -LiteralPath $VsWhere -PathType Leaf)) {
        throw '找不到 zig 或 Visual Studio C 编译器，无法运行通用控制器回归。'
    }
    $VsRoot = & $VsWhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    $VsDevCmd = Join-Path $VsRoot 'Common7\Tools\VsDevCmd.bat'
    $Compile = 'call "{0}" -arch=x64 >nul && cl /nologo /utf-8 /std:c11 /W4 /WX /DCONTROL_MANAGER_TEST=1 /FIControlMgrTestCritical.h /FIControlRuntimeTestPlatform.h /I"{1}" /I"{2}" /I"{3}" /I"{4}" /I"{5}" /I"{6}" /I"{7}" "{8}" "{9}" "{10}" /Fe"{11}"' -f `
        $VsDevCmd,
        (Join-Path $RepoRoot 'tools\tests'),
        (Join-Path $RepoRoot 'shared\application\robot'),
        (Join-Path $RepoRoot 'shared\application\input'),
        (Join-Path $RepoRoot 'shared\application\motors'),
        (Join-Path $RepoRoot 'shared\application\services\diagnostics'),
        (Join-Path $RepoRoot 'shared\hal'),
        (Join-Path $RepoRoot 'shared\components\support'),
        $Sources[0], $Sources[1], $Sources[2], $Output
    Push-Location -LiteralPath $BuildDir
    try {
        & $env:ComSpec /d /c $Compile
        if ($LASTEXITCODE -ne 0) {
            throw "通用控制器回归编译失败，退出码 $LASTEXITCODE"
        }
    }
    finally {
        Pop-Location
    }
}
& $Output
if ($LASTEXITCODE -ne 0) {
    throw "通用控制器回归失败，退出码 $LASTEXITCODE"
}
