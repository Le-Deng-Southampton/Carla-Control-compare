$ErrorActionPreference = "Stop"
$ExperimentArgs = $args

function Test-TcpPort {
    param(
        [Parameter(Mandatory = $true)]
        [string]$HostName,

        [Parameter(Mandatory = $true)]
        [int]$Port,

        [int]$TimeoutMilliseconds = 500
    )

    $client = New-Object System.Net.Sockets.TcpClient
    try {
        $result = $client.BeginConnect($HostName, $Port, $null, $null)
        if (-not $result.AsyncWaitHandle.WaitOne($TimeoutMilliseconds, $false)) {
            return $false
        }

        $client.EndConnect($result)
        return $true
    }
    catch {
        return $false
    }
    finally {
        $client.Close()
    }
}

function Wait-ForCarlaServer {
    param(
        [int]$TimeoutSeconds = 90
    )

    $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
    while ((Get-Date) -lt $deadline) {
        if (Test-TcpPort -HostName "localhost" -Port 2000) {
            return $true
        }
        Start-Sleep -Seconds 2
    }

    return $false
}

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$carlaExe = Join-Path $root "CarlaUE4\Binaries\Win64\CarlaUE4-Win64-Shipping.exe"
$projectLauncher = Join-Path $root "PythonAPI\my_control_project\scripts\run_my_control.ps1"
$logDir = Join-Path $root "PythonAPI\my_control_project\log"

if (-not (Test-Path -LiteralPath $carlaExe)) {
    throw "CARLA executable was not found: $carlaExe"
}

if (-not (Test-Path -LiteralPath $projectLauncher)) {
    throw "Project launcher was not found: $projectLauncher"
}

New-Item -ItemType Directory -Force -Path $logDir | Out-Null
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$transcriptPath = Join-Path $logDir "console_desktop_$timestamp.txt"

Start-Transcript -Path $transcriptPath -Append | Out-Null
$exitCode = 0
try {
    Write-Host "[START] $(Get-Date -Format o)"
    Write-Host "[ENTRY] Desktop shortcut target: $root\Start-CARLA-Project.bat"
    Write-Host "[LOG] $transcriptPath"

    $serverReady = Test-TcpPort -HostName "localhost" -Port 2000
    if ($serverReady) {
        Write-Host "[CARLA] Existing simulator detected on localhost:2000."
    }
    else {
        Write-Host "[CARLA] Starting simulator..."
        Start-Process -FilePath $carlaExe -WorkingDirectory $root | Out-Null
        if (-not (Wait-ForCarlaServer -TimeoutSeconds 90)) {
            throw "CARLA did not open localhost:2000 within 90 seconds. Check the simulator window and try again."
        }
        Write-Host "[CARLA] Simulator is ready on localhost:2000."
    }

    Write-Host "[PROJECT] Launching controller comparison..."
    & $projectLauncher @ExperimentArgs
    $exitCode = $LASTEXITCODE
    if ($null -eq $exitCode) {
        $exitCode = 0
    }

    Write-Host "[END] $(Get-Date -Format o) exit=$exitCode"
}
catch {
    Write-Host "[ERROR] $($_.Exception.Message)"
    $exitCode = 1
}
finally {
    Stop-Transcript | Out-Null
}

exit $exitCode
