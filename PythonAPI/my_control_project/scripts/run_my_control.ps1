$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectRoot = Split-Path -Parent $scriptDir
$existingPythonPath = $env:PYTHONPATH
$commonScript = Join-Path (Split-Path -Parent $projectRoot) "scripts\common.ps1"

if (-not (Test-Path -LiteralPath $commonScript)) {
    throw "CARLA Python helper script was not found: $commonScript"
}

. $commonScript
$conda = Get-CondaCommand

Push-Location $projectRoot
try {
    if ([string]::IsNullOrWhiteSpace($existingPythonPath)) {
        $env:PYTHONPATH = $projectRoot
    }
    else {
        $env:PYTHONPATH = "$projectRoot;$existingPythonPath"
    }
    & $conda run -n carla37 python $projectRoot\run_my_control.py @args
    if ($LASTEXITCODE -ne 0) {
        throw "run_my_control.py failed with exit code $LASTEXITCODE"
    }
}
finally {
    $env:PYTHONPATH = $existingPythonPath
    Pop-Location
}
