function Get-TestRobotConfig {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Project
    )

    $repoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
    $python = Join-Path $repoRoot 'local\cache\zephyrproject\.venv\Scripts\python.exe'
    $generator = Join-Path $repoRoot 'tools\config\RobotConfigGen.py'
    $output = Join-Path $repoRoot (Join-Path 'local\build\host-tests\config' $Project)

    if (-not (Test-Path -LiteralPath $python -PathType Leaf)) {
        throw "找不到项目 Python：$python"
    }
    if (-not (Test-Path -LiteralPath $generator -PathType Leaf)) {
        throw "找不到车型配置生成器：$generator"
    }

    & $python -X utf8 $generator generate --target $Project --out $output | Out-Null
    if ($LASTEXITCODE -ne 0) {
        throw "生成 $Project 的宿主测试配置失败，退出码 $LASTEXITCODE"
    }
    if (-not (Test-Path -LiteralPath (Join-Path $output 'RobotTargetConfig.h') -PathType Leaf)) {
        throw "生成目录缺少 RobotTargetConfig.h：$output"
    }

    return $output
}
