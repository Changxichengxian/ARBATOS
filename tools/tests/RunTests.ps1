param(
    [string]$Name,
    [switch]$List
)

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$BuildDir = Join-Path $RepoRoot 'local\build\host-tests'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null

$PowerShellTests = @(Get-ChildItem -LiteralPath $PSScriptRoot -File -Filter 'Test*.ps1' |
    Sort-Object Name)
$PythonTests = @(Get-ChildItem -LiteralPath $PSScriptRoot -File -Filter 'Test*.py' |
    Sort-Object Name)

if ($List) {
    Write-Host '可运行的主机测试：'
    foreach ($Test in $PowerShellTests) {
        Write-Host "  $($Test.BaseName)"
    }
    foreach ($Test in $PythonTests) {
        Write-Host "  $($Test.BaseName)"
    }
    exit 0
}

$SelectedTests = @()
if ([string]::IsNullOrWhiteSpace($Name)) {
    $SelectedTests = $PowerShellTests
    $SelectedPythonTests = $PythonTests
}
else {
    $NormalizedName = [IO.Path]::GetFileNameWithoutExtension($Name)
    $ExpectedName = if ($NormalizedName.StartsWith('Test')) {
        $NormalizedName
    }
    else {
        'Test' + $NormalizedName
    }
    $SelectedTests = @($PowerShellTests | Where-Object { $_.BaseName -ieq $ExpectedName })
    $SelectedPythonTests = @($PythonTests | Where-Object { $_.BaseName -ieq $ExpectedName })
    if ($SelectedTests.Count -eq 0 -and $SelectedPythonTests.Count -eq 0) {
        throw "找不到测试 [$($Name)]。请先运行 .\RunTests.ps1 -List 查看可用名称。"
    }
}

$Timestamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$Transcript = Join-Path $BuildDir "RunTests-$Timestamp.log"
$Failures = @()
$OriginalPath = $env:Path
Start-Transcript -LiteralPath $Transcript | Out-Null
try {
    # 已通过 WinGet 安装但当前终端尚未刷新 PATH 时，只为本次测试补上位置。
    if ($SelectedTests.Count -gt 0 -and -not (Get-Command zig -ErrorAction SilentlyContinue) -and $env:LOCALAPPDATA) {
        $WingetLinks = Join-Path $env:LOCALAPPDATA 'Microsoft\WinGet\Links'
        if (Test-Path -LiteralPath (Join-Path $WingetLinks 'zig.exe') -PathType Leaf) {
            $env:Path = "$WingetLinks;$env:Path"
        }
    }
    foreach ($Test in $SelectedTests) {
        Write-Host ''
        Write-Host "[运行] $($Test.BaseName)"
        try {
            $global:LASTEXITCODE = 0
            & $Test.FullName
            if ($LASTEXITCODE -ne 0) {
                throw "退出码 $LASTEXITCODE"
            }
            Write-Host "[通过] $($Test.BaseName)"
        }
        catch {
            $Failures += "$($Test.BaseName): $($_.Exception.Message)"
            Write-Host "[失败] $($Test.BaseName): $($_.Exception.Message)" -ForegroundColor Red
        }
    }

    foreach ($PythonTest in $SelectedPythonTests) {
        Write-Host ''
        Write-Host "[运行] $($PythonTest.BaseName)"
        try {
            $VenvPython = Join-Path $RepoRoot 'local\cache\zephyrproject\.venv\Scripts\python.exe'
            if (Test-Path -LiteralPath $VenvPython) {
                $Python = $VenvPython
            }
            else {
                $PythonCommand = Get-Command python -ErrorAction SilentlyContinue
                if ($null -eq $PythonCommand) {
                    throw '找不到项目虚拟环境或 PATH 中的 python。'
                }
                $Python = $PythonCommand.Source
            }
            & $Python -X utf8 $PythonTest.FullName
            if ($LASTEXITCODE -ne 0) {
                throw "退出码 $LASTEXITCODE"
            }
            Write-Host "[通过] $($PythonTest.BaseName)"
        }
        catch {
            $Failures += "$($PythonTest.BaseName): $($_.Exception.Message)"
            Write-Host "[失败] $($PythonTest.BaseName): $($_.Exception.Message)" -ForegroundColor Red
        }
    }

    if ($Failures.Count -gt 0) {
        Write-Host ''
        Write-Host "主机测试失败：$($Failures.Count) 项。日志：$Transcript" -ForegroundColor Red
        foreach ($Failure in $Failures) {
            Write-Host "  $Failure" -ForegroundColor Red
        }
        exit 1
    }

    Write-Host ''
    Write-Host "主机测试全部通过。日志：$Transcript" -ForegroundColor Green
}
finally {
    $env:Path = $OriginalPath
    Stop-Transcript | Out-Null
}
