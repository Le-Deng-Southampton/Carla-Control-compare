[CmdletBinding()]
param(
    [string]$OutputRoot = "",
    [int]$RpcPort = 2000,
    [ValidateSet("full", "smoke")]
    [string]$ScenarioSet = "full",
    [string[]]$VariantFilter = @("*"),
    [switch]$SkipAnalysis
)

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectRoot = Split-Path -Parent $scriptDir
$matrixRunner = Join-Path $scriptDir "run_controller_evaluation.ps1"
$analysisTool = Join-Path $projectRoot "experiment\internal_module_ablation_analysis.py"
$commonScript = Join-Path (Split-Path -Parent $projectRoot) "scripts\common.ps1"
. $commonScript
$conda = Get-CondaCommand
$carlaRoot = Split-Path -Parent (Split-Path -Parent $projectRoot)
$carlaLauncher = Join-Path $carlaRoot "CarlaUE4.exe"
if (-not (Test-Path -LiteralPath $carlaLauncher -PathType Leaf)) {
    $carlaLauncher = Join-Path $carlaRoot "CarlaUE4\Binaries\Win64\CarlaUE4-Win64-Shipping.exe"
}
if ([string]::IsNullOrWhiteSpace($OutputRoot)) {
    $OutputRoot = Join-Path $projectRoot ("log\internal_module_ablation_90_" + (Get-Date -Format "yyyyMMdd_HHmmss"))
}
$OutputRoot = [IO.Path]::GetFullPath($OutputRoot)
New-Item -ItemType Directory -Force -Path $OutputRoot | Out-Null

$pidFull = @(
    "--pid-curvature-feedforward-gain", "0.20",
    "--pid-derivative-filter-alpha", "0.50"
)
$lqrFull = @()
$mpcFull = @(
    "--mpc-model-type", "dynamic_bicycle",
    "--mpc-enable-adaptive-horizon",
    "--mpc-enable-event-trigger",
    "--mpc-enable-small-error-steer-deadband"
)

$variants = @(
    [pscustomobject]@{ Controller = "pid"; Variant = "full"; Module = "all"; Args = $pidFull },
    [pscustomobject]@{ Controller = "pid"; Variant = "without_feedforward"; Module = "curvature_feedforward"; Args = @("--pid-curvature-feedforward-gain", "0.0", "--pid-derivative-filter-alpha", "0.50") },
    [pscustomobject]@{ Controller = "pid"; Variant = "without_derivative_filter"; Module = "derivative_filter"; Args = @("--pid-curvature-feedforward-gain", "0.20", "--pid-derivative-filter-alpha", "1.0") },
    [pscustomobject]@{ Controller = "pid"; Variant = "without_integral_protection"; Module = "integral_limit_and_separation"; Args = $pidFull + @("--pid-disable-integral-protection") },
    [pscustomobject]@{ Controller = "pid"; Variant = "without_anti_windup"; Module = "conditional_anti_windup"; Args = $pidFull + @("--pid-disable-anti-windup") },

    [pscustomobject]@{ Controller = "lqr"; Variant = "full"; Module = "all"; Args = $lqrFull },
    [pscustomobject]@{ Controller = "lqr"; Variant = "without_preview"; Module = "curvature_sequence_preview"; Args = @("--lqr-curvature-preview-blend", "0.0") },
    [pscustomobject]@{ Controller = "lqr"; Variant = "without_dynamic_envelope"; Module = "lateral_acceleration_envelope"; Args = @("--lqr-max-lateral-accel", "0.0") },
    [pscustomobject]@{ Controller = "lqr"; Variant = "without_state_filter"; Module = "state_derivative_filter"; Args = @("--lqr-derivative-alpha", "1.0") },
    [pscustomobject]@{ Controller = "lqr"; Variant = "without_curvature_filter"; Module = "curvature_filter"; Args = @("--lqr-curvature-alpha", "1.0") },
    [pscustomobject]@{ Controller = "lqr"; Variant = "without_feedforward"; Module = "curvature_feedforward"; Args = @("--lqr-feedforward-gain", "0.0") },
    [pscustomobject]@{ Controller = "lqr"; Variant = "without_guards"; Module = "turn_in_and_inside_error_guards"; Args = @("--lqr-turn-in-rate-scale", "1.0", "--lqr-inside-error-feedforward-min-scale", "1.0") },

    [pscustomobject]@{ Controller = "mpc"; Variant = "full"; Module = "all"; Args = $mpcFull },
    [pscustomobject]@{ Controller = "mpc"; Variant = "without_dynamic_model"; Module = "dynamic_bicycle_model"; Args = @("--mpc-model-type", "kinematic", "--mpc-enable-adaptive-horizon", "--mpc-enable-event-trigger", "--mpc-enable-small-error-steer-deadband") },
    [pscustomobject]@{ Controller = "mpc"; Variant = "without_adaptive_horizon"; Module = "adaptive_horizon"; Args = @("--mpc-model-type", "dynamic_bicycle", "--mpc-enable-event-trigger", "--mpc-enable-small-error-steer-deadband") },
    [pscustomobject]@{ Controller = "mpc"; Variant = "without_curvature_conditioning"; Module = "curvature_filter_and_step_limit"; Args = $mpcFull + @("--mpc-curvature-filter-alpha", "1.0", "--mpc-curvature-step-limit", "0.0") },
    [pscustomobject]@{ Controller = "mpc"; Variant = "without_feedforward"; Module = "curvature_reference_steer"; Args = $mpcFull + @("--mpc-curvature-feedforward-gain", "0.0") },
    [pscustomobject]@{ Controller = "mpc"; Variant = "without_steer_accel_penalty"; Module = "steer_acceleration_penalty"; Args = $mpcFull + @("--mpc-r-steer-accel", "0.0") },
    [pscustomobject]@{ Controller = "mpc"; Variant = "without_dynamic_envelope"; Module = "lateral_acceleration_envelope"; Args = $mpcFull + @("--mpc-max-lateral-accel", "0.0") },
    [pscustomobject]@{ Controller = "mpc"; Variant = "without_event_trigger"; Module = "event_trigger_reuse"; Args = @("--mpc-model-type", "dynamic_bicycle", "--mpc-enable-adaptive-horizon", "--mpc-enable-small-error-steer-deadband") },
    [pscustomobject]@{ Controller = "mpc"; Variant = "without_deadband"; Module = "small_error_steer_deadband"; Args = @("--mpc-model-type", "dynamic_bicycle", "--mpc-enable-adaptive-horizon", "--mpc-enable-event-trigger") }
)

$selected = @($variants | Where-Object {
    $key = "$($_.Controller)/$($_.Variant)"
    @($VariantFilter | Where-Object { $key -like $_ }).Count -gt 0
})
if ($selected.Count -eq 0) { throw "VariantFilter selected no module variants." }

$manifest = [ordered]@{
    version = 1
    scenario_set = $ScenarioSet
    expected_cases_per_variant = if ($ScenarioSet -eq "full") { 90 } else { 2 }
    bootstrap_iterations = 5000
    classification = [ordered]@{
        primary_metric = "normalized_iae_e_y_m"
        practical_effect_threshold_pct = 3.0
        positive = "hard-gate rate improves, or lateral IAE improves by at least 3% with CI95 above zero"
        negative = "hard-gate rate regresses, or lateral IAE regresses by at least 3% with CI95 below zero"
        no_significant_effect = "neither positive nor negative rule is met"
    }
    variants = @($variants | ForEach-Object {
        [ordered]@{
            controller = $_.Controller
            variant = $_.Variant
            removed_module = $_.Module
            additional_run_args = @($_.Args)
            output_dir = Join-Path $OutputRoot "$($_.Controller)_$($_.Variant)"
        }
    })
}
$manifestPath = Join-Path $OutputRoot "ablation_manifest.json"
$manifest | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $manifestPath -Encoding utf8

$expectedCases = [int]$manifest.expected_cases_per_variant
function Test-CarlaRpcAvailable {
    param([int]$Port)

    try {
        $client = [Net.Sockets.TcpClient]::new()
        $async = $client.BeginConnect("127.0.0.1", $Port, $null, $null)
        if ($async.AsyncWaitHandle.WaitOne(1000)) {
            $client.EndConnect($async)
            $client.Close()
            return $true
        }
        $client.Close()
    }
    catch {
        return $false
    }
    return $false
}

function Ensure-CarlaAvailable {
    param([int]$Port, [int]$TimeoutSeconds = 120)

    if (Test-CarlaRpcAvailable -Port $Port) { return }
    if (-not (Test-Path -LiteralPath $carlaLauncher -PathType Leaf)) {
        throw "CARLA launcher was not found: $carlaLauncher"
    }

    $server = Get-Process -Name CarlaUE4, CarlaUE4-Win64-Shipping -ErrorAction SilentlyContinue
    if (-not $server) {
        Write-Host "Starting CARLA simulator on RPC port $Port..."
        Start-Process -FilePath $carlaLauncher -ArgumentList "-carla-rpc-port=$Port", "-quality-level=Low" -WorkingDirectory $carlaRoot -WindowStyle Hidden
    }

    $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
    while ((Get-Date) -lt $deadline) {
        if (Test-CarlaRpcAvailable -Port $Port) { return }
        Start-Sleep -Seconds 2
    }
    throw "CARLA RPC port $Port was not reachable within $TimeoutSeconds seconds."
}

function Test-AllProfilesComplete {
    return @($variants | Where-Object {
        $statusPath = Join-Path $OutputRoot "$($_.Controller)_$($_.Variant)\run_status.csv"
        if (-not (Test-Path -LiteralPath $statusPath)) {
            $false
        }
        else {
            $statusRows = @(Import-Csv -LiteralPath $statusPath)
            $invalidRows = @($statusRows | Where-Object { $_.status -ne "ok" })
            $statusRows.Count -eq $expectedCases -and $invalidRows.Count -eq 0
        }
    }).Count -eq $variants.Count
}

do {
    foreach ($variant in $selected) {
        $variantOutput = Join-Path $OutputRoot "$($variant.Controller)_$($variant.Variant)"
        Write-Host "Running 90-case module profile $($variant.Controller)/$($variant.Variant)"
        $invoke = @{
            OutputRoot = $variantOutput
            ScenarioSet = $ScenarioSet
            Controllers = @($variant.Controller)
            AllowControllerSubset = $true
            RpcPort = $RpcPort
            AdditionalRunArgs = @($variant.Args)
            Resume = $true
        }
        $matrixExitCode = 0
        try {
            Ensure-CarlaAvailable -Port $RpcPort
            & $matrixRunner @invoke
            $matrixExitCode = $LASTEXITCODE
        }
        catch {
            $matrixExitCode = 1
            if ($SkipAnalysis) {
                Write-Warning "$($_.Exception.Message) The profile will be retried on the next resume pass: $($variant.Controller)/$($variant.Variant)"
            }
            else {
                throw
            }
        }
        if ($matrixExitCode -ne 0 -and -not $SkipAnalysis) {
            throw "Module matrix failed with exit code ${matrixExitCode}: $($variant.Controller)/$($variant.Variant)"
        }
    }

    $allComplete = Test-AllProfilesComplete
    if (-not $allComplete -and $SkipAnalysis) {
        Write-Host "Incomplete or failed cases remain; starting another resume pass in 10 seconds."
        Start-Sleep -Seconds 10
    }
} while ($SkipAnalysis -and -not $allComplete)

if ($allComplete -and -not $SkipAnalysis) {
    & $conda run -n carla37 python $analysisTool --manifest $manifestPath --output-dir $OutputRoot
    if ($LASTEXITCODE -ne 0) { throw "Module ablation analysis failed with exit code $LASTEXITCODE" }
}
elseif ($allComplete) {
    $completion = [ordered]@{
        completed_at_utc = [DateTime]::UtcNow.ToString("o")
        scenario_set = $ScenarioSet
        profiles_complete = $variants.Count
        cases_per_profile = $manifest.expected_cases_per_variant
        analysis_skipped = $true
    }
    $completion | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $OutputRoot "TESTS_COMPLETE.json") -Encoding utf8
    Write-Host "All test profiles complete; analysis skipped by request."
}
else {
    Write-Host "Selected profiles complete; run remaining profiles before aggregate analysis."
}

Write-Host "90-case internal-module ablation root: $OutputRoot"
