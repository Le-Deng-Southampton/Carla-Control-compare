$ErrorActionPreference = "Stop"

function Get-CondaCommand {
    $userConda = Join-Path $env:USERPROFILE "miniconda3\Scripts\conda.exe"
    if (Test-Path $userConda) {
        return $userConda
    }

    $pathConda = Get-Command "conda" -ErrorAction SilentlyContinue
    if ($pathConda) {
        return $pathConda.Source
    }

    throw "conda was not found. Install Miniconda or run PythonAPI\setup_carla37.ps1 after conda is available."
}

function Invoke-CondaCommand {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Conda,

        [Parameter(ValueFromRemainingArguments = $true)]
        [string[]]$Arguments
    )

    & $Conda @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "conda failed: $($Arguments -join ' ')"
    }
}
