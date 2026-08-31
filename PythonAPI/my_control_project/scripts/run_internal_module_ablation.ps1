[CmdletBinding()]
param(
    [string]$OutputRoot = "",
    [int]$RpcPort = 2000
)

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectRoot = Split-Path -Parent $scriptDir
$runner = Join-Path $scriptDir "run_my_control.ps1"
$logRoot = Join-Path $projectRoot "log"
if ([string]::IsNullOrWhiteSpace($OutputRoot)) {
    $OutputRoot = Join-Path $logRoot ("internal_module_ablation_" + (Get-Date -Format "yyyyMMdd_HHmmss"))
}
$OutputRoot = [IO.Path]::GetFullPath($OutputRoot)
New-Item -ItemType Directory -Force -Path $OutputRoot | Out-Null

$common = @(
    "--evaluation-headless",
    "--planner-mode", "legacy",
    "--planner-fallback", "error",
    "--speed-planner-mode", "off",
    "--speed-planner-limit-profile", "global",
    "--target-speed", "50",
    "--route-min-length-m", "250",
    "--route-shape", "curvy",
    "--map-name", "Town05_Opt",
    "--seed", "26050301",
    "--error-provider", "ground_truth",
    "--carla-port", [string]$RpcPort
)

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

$rawResultsPath = Join-Path $OutputRoot "raw_results.csv"
$records = if (Test-Path -LiteralPath $rawResultsPath) {
    @(Import-Csv -LiteralPath $rawResultsPath)
} else {
    @()
}
foreach ($variant in $variants) {
    $completed = $records | Where-Object {
        $_.controller -eq $variant.Controller -and $_.variant -eq $variant.Variant
    } | Select-Object -First 1
    if ($null -ne $completed) {
        Write-Host ("Skipping completed {0}/{1}" -f $variant.Controller, $variant.Variant)
        continue
    }
    Write-Host ("Running {0}/{1}" -f $variant.Controller, $variant.Variant)
    $started = Get-Date
    $caseId = "internal_ablation_{0}_{1}" -f $variant.Controller, $variant.Variant
    $args = $common + @(
        "--evaluation-case-id", $caseId,
        "--controllers", $variant.Controller
    ) + $variant.Args
    & $runner @args
    if ($LASTEXITCODE -ne 0) {
        throw "Ablation run failed: $($variant.Controller)/$($variant.Variant)"
    }
    $run = Get-ChildItem -LiteralPath $logRoot -Directory |
        Where-Object { $_.Name -like "run_*_seed_26050301_curvy" -and $_.LastWriteTime -ge $started } |
        Sort-Object LastWriteTime -Descending |
        Select-Object -First 1
    if ($null -eq $run) {
        throw "Could not locate run output for $($variant.Controller)/$($variant.Variant)"
    }
    $summaryPath = Join-Path $run.FullName "summary.csv"
    $summary = Import-Csv -LiteralPath $summaryPath |
        Where-Object { $_.controller -eq $variant.Controller } |
        Select-Object -First 1
    if ($null -eq $summary) {
        throw "Missing summary row for $($variant.Controller)/$($variant.Variant)"
    }
    $record = [ordered]@{
        controller = $variant.Controller
        variant = $variant.Variant
        removed_module = $variant.Module
        run_dir = $run.FullName
    }
    foreach ($property in $summary.PSObject.Properties) {
        $record[$property.Name] = $property.Value
    }
    $records += [pscustomobject]$record
    $records | Export-Csv -LiteralPath $rawResultsPath -NoTypeInformation -Encoding utf8
}

$comparisons = @()
foreach ($controller in @("pid", "lqr", "mpc")) {
    $full = $records | Where-Object { $_.controller -eq $controller -and $_.variant -eq "full" } | Select-Object -First 1
    foreach ($row in ($records | Where-Object { $_.controller -eq $controller -and $_.variant -ne "full" })) {
        $fullIae = [double]$full.normalized_iae_e_y_m
        $ablatedIae = [double]$row.normalized_iae_e_y_m
        $trackingImprovement = if ([math]::Abs($ablatedIae) -gt 1e-12) {
            100.0 * ($ablatedIae - $fullIae) / [math]::Abs($ablatedIae)
        } else { 0.0 }
        $moduleSafetyRegression = (
            [int]$full.collision_count -gt [int]$row.collision_count -or
            [int]$full.lane_boundary_violation_count -gt [int]$row.lane_boundary_violation_count -or
            [double]$full.route_completion_pct + 0.01 -lt [double]$row.route_completion_pct
        )
        $ablationSafetyRegression = (
            [int]$row.collision_count -gt [int]$full.collision_count -or
            [int]$row.lane_boundary_violation_count -gt [int]$full.lane_boundary_violation_count -or
            [double]$row.route_completion_pct + 0.01 -lt [double]$full.route_completion_pct
        )
        $classification = if ($moduleSafetyRegression -or $trackingImprovement -lt -3.0) {
            "negative"
        } elseif ($ablationSafetyRegression -or $trackingImprovement -gt 3.0) {
            "positive"
        } else {
            "no_significant_effect"
        }
        $comparisons += [pscustomobject][ordered]@{
            controller = $controller
            module = $row.removed_module
            classification = $classification
            lateral_iae_improvement_pct = $trackingImprovement
            heading_iae_full = [double]$full.normalized_iae_e_psi_deg
            heading_iae_without_module = [double]$row.normalized_iae_e_psi_deg
            p95_steer_delta_full = [double]$full.p95_abs_steer_delta
            p95_steer_delta_without_module = [double]$row.p95_abs_steer_delta
            p95_lateral_jerk_full = [double]$full.p95_abs_lateral_jerk
            p95_lateral_jerk_without_module = [double]$row.p95_abs_lateral_jerk
            runtime_p95_ms_full = [double]$full.controller_runtime_p95_ms
            runtime_p95_ms_without_module = [double]$row.controller_runtime_p95_ms
            module_safety_regression = $moduleSafetyRegression
            ablation_safety_regression = $ablationSafetyRegression
            full_run_dir = $full.run_dir
            ablated_run_dir = $row.run_dir
        }
    }
}
$comparisons | Export-Csv -LiteralPath (Join-Path $OutputRoot "module_effects.csv") -NoTypeInformation -Encoding utf8
Write-Host "Ablation results: $OutputRoot"
