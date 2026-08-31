[CmdletBinding(PositionalBinding = $false)]
param(
    [switch]$HealthCheck,
    [int]$HealthPort = 2000,
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$RunArgs
)

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
    if ($HealthCheck) {
        & $conda run -n carla37 python $projectRoot\carla_health.py --port $HealthPort --timeout 2.0
    }
    else {
        & $conda run -n carla37 python $projectRoot\run_my_control.py @RunArgs
    }
    if ($LASTEXITCODE -ne 0) {
        $commandName = if ($HealthCheck) { "carla_health.py" } else { "run_my_control.py" }
        throw "$commandName failed with exit code $LASTEXITCODE"
    }
}
finally {
    $env:PYTHONPATH = $existingPythonPath
    Pop-Location
}
