$ErrorActionPreference = "Stop"

function Get-CondaCommand {
    $candidates = @(
        $env:CONDA_EXE,
        (Join-Path $env:USERPROFILE "miniconda3\Scripts\conda.exe"),
        "D:\miniconda3\Scripts\conda.exe"
    ) | Where-Object { -not [string]::IsNullOrWhiteSpace($_) }
    foreach ($candidate in $candidates) {
        if (Test-Path -LiteralPath $candidate -PathType Leaf) {
            return $candidate
        }
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
