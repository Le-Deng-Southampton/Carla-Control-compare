$ErrorActionPreference = "Stop"

function Get-CondaCommand {
    $userConda = Join-Path $env:USERPROFILE "miniconda3\Scripts\conda.exe"
    if (Test-Path $userConda) {
        return $userConda
    }

    return "conda"
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
