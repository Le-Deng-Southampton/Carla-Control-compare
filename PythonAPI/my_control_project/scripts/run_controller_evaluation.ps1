param(
    [string]$OutputRoot = "",
    [string]$ManifestPath = "",
    [switch]$Resume,
    [ValidateSet("full", "smoke")]
    [string]$ScenarioSet = "full",
    [string[]]$Controllers = @("pid", "lqr", "mpc"),
    [switch]$AllowControllerSubset,
    [int]$RpcPort = 2000,
    [string]$SingleRunScript = "",
    [string[]]$AdditionalRunArgs = @(),
    [string]$OnlyCaseId = "",
    [ValidateSet("optimized", "authoritative_baseline")]
    [string]$ImplementationLabel = "optimized"
)

$ErrorActionPreference = "Stop"

# Full matrix: 10 base scenarios x 3 seeds x 2 layers = 60 experiments.
# Robustness matrix: 3 stress scenarios x 10 perturbations = 30 experiments.
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectRoot = Split-Path -Parent $scriptDir
if ([string]::IsNullOrWhiteSpace($SingleRunScript)) {
    $SingleRunScript = Join-Path $scriptDir "run_my_control.ps1"
}
$singleRun = [IO.Path]::GetFullPath($SingleRunScript)
if (-not (Test-Path -LiteralPath $singleRun -PathType Leaf)) {
    throw "Single-run controller script was not found: $singleRun"
}
$logRoot = Join-Path $projectRoot "log"
$perturbationTool = Join-Path $projectRoot "experiment\perturbations.py"
$aggregateTool = Join-Path $projectRoot "experiment\evaluation_aggregate.py"
$reportTool = Join-Path $projectRoot "experiment\evaluation_report.py"

if ([string]::IsNullOrWhiteSpace($OutputRoot)) {
    $OutputRoot = Join-Path $logRoot ("controller_evaluation_" + (Get-Date -Format "yyyyMMdd_HHmmss"))
}
$OutputRoot = [IO.Path]::GetFullPath($OutputRoot)
New-Item -ItemType Directory -Force -Path $OutputRoot | Out-Null
if ([string]::IsNullOrWhiteSpace($ManifestPath)) {
    $ManifestPath = Join-Path $OutputRoot "evaluation_manifest.json"
}
$statusPath = Join-Path $OutputRoot "run_status.csv"
$controllerSet = (@($Controllers | Sort-Object -Unique) -join ",")
if (-not $AllowControllerSubset -and $controllerSet -ne "lqr,mpc,pid") {
    throw "A fair evaluation requires exactly the pid, lqr and mpc controllers."
}
if ($AllowControllerSubset) {
    $unsupported = @($Controllers | Where-Object { $_ -notin @("pid", "lqr", "mpc") })
    if ($Controllers.Count -eq 0 -or $unsupported.Count -gt 0) {
        throw "Controller subset must contain one or more of: pid, lqr, mpc."
    }
}

function New-EvaluationCase {
    param([object]$Scenario, [int]$Seed, [string]$Layer, [object]$Perturbation)

    $id = "{0}_{1}_{2}" -f $Layer, $Scenario.Name, $Seed
    return [pscustomobject][ordered]@{
        evaluation_case_id = $id
        scenario = $Scenario.Name
        speed_kmh = [int]$Scenario.Speed
        route_length_m = [int]$Scenario.Length
        route_shape = $Scenario.Route
        map_name = $Scenario.Map
        seed = $Seed
        layer = $Layer
        perturbation = $Perturbation
    }
}

$baseScenarios = @(
    [pscustomobject]@{ Name = "urban_30_true_straight"; Speed = 30; Length = 200; Route = "true_straight"; Map = "Town04"; Seeds = @(26030101, 26030102, 26030103) },
    [pscustomobject]@{ Name = "urban_30_curvy"; Speed = 30; Length = 200; Route = "curvy"; Map = "Town05_Opt"; Seeds = @(26030201, 26030202, 26030203) },
    [pscustomobject]@{ Name = "urban_50_true_straight"; Speed = 50; Length = 250; Route = "true_straight"; Map = "Town04"; Seeds = @(26050101, 26050102, 26050103) },
    [pscustomobject]@{ Name = "urban_50_s_curve"; Speed = 50; Length = 250; Route = "s_curve"; Map = "Town04"; Seeds = @(26050201, 26050202, 26050203) },
    [pscustomobject]@{ Name = "urban_50_curvy"; Speed = 50; Length = 250; Route = "curvy"; Map = "Town05_Opt"; Seeds = @(26050301, 26050302, 26050103) },
    [pscustomobject]@{ Name = "road_70_true_straight"; Speed = 70; Length = 300; Route = "true_straight"; Map = "Town04"; Seeds = @(26070101, 26070102, 26070103) },
    [pscustomobject]@{ Name = "road_70_s_curve"; Speed = 70; Length = 300; Route = "s_curve"; Map = "Town04"; Seeds = @(26070201, 26070202, 26070203) },
    [pscustomobject]@{ Name = "road_70_gentle_curve"; Speed = 70; Length = 300; Route = "gentle_curve"; Map = "Town05_Opt"; Seeds = @(26070301, 26070302, 26070303) },
    [pscustomobject]@{ Name = "highway_120_true_straight"; Speed = 120; Length = 500; Route = "true_straight"; Map = "Town04"; Seeds = @(26120101, 26120102, 26120103) },
    [pscustomobject]@{ Name = "highway_90_s_curve"; Speed = 90; Length = 300; Route = "s_curve"; Map = "Town04"; Seeds = @(26070201, 26070202, 26070203) }
)

$stressScenarios = @(
    [pscustomobject]@{ Name = "stress_30_curvy"; Speed = 30; Length = 200; Route = "curvy"; Map = "Town05_Opt"; BaseSeed = 26300100 },
    [pscustomobject]@{ Name = "stress_70_s_curve"; Speed = 70; Length = 300; Route = "s_curve"; Map = "Town04"; BaseSeed = 26700100 },
    [pscustomobject]@{ Name = "stress_90_s_curve"; Speed = 90; Length = 300; Route = "s_curve"; Map = "Town04"; BaseSeed = 26900100 }
)

function Get-FullCases {
    $baseCases = @(
        foreach ($scenario in $baseScenarios) {
            foreach ($seed in $scenario.Seeds) {
                New-EvaluationCase $scenario $seed "controller_only" $null
                New-EvaluationCase $scenario $seed "integrated" $null
            }
        }
    )
    $stressCases = @(
        foreach ($scenario in $stressScenarios) {
            0..9 | ForEach-Object {
                $seed = $scenario.BaseSeed + $_
                $perturbationJson = & python $perturbationTool --seed $seed --json
                if ($LASTEXITCODE -ne 0) { throw "Failed to generate perturbation seed $seed" }
                $perturbation = $perturbationJson | ConvertFrom-Json
                New-EvaluationCase $scenario $seed "integrated_stress" $perturbation
            }
        }
    )
    if ($baseCases.Count -ne 60) { throw "Expected 60 base cases, got $($baseCases.Count)" }
    if ($stressCases.Count -ne 30) { throw "Expected 30 stress cases, got $($stressCases.Count)" }
    return @($baseCases + $stressCases | Sort-Object map_name, scenario, seed, layer)
}

function Get-SmokeCases {
    $straight = [pscustomobject]@{ Name = "smoke_30_straight_controller_only"; Speed = 30; Length = 200; Route = "straight"; Map = "Town04" }
    $sCurve = [pscustomobject]@{ Name = "smoke_70_s_curve_integrated"; Speed = 70; Length = 300; Route = "s_curve"; Map = "Town04" }
    $cases = @(
        New-EvaluationCase $straight 26050101 "controller_only" $null
        New-EvaluationCase $sCurve 26070102 "integrated" $null
    )
    if ($cases.Count -ne 2) { throw "Smoke matrix must contain exactly two experiments" }
    return $cases
}

$cases = if ($ScenarioSet -eq "smoke") { @(Get-SmokeCases) } else { @(Get-FullCases) }
if (-not [string]::IsNullOrWhiteSpace($OnlyCaseId)) {
    $cases = @($cases | Where-Object { $_.evaluation_case_id -eq $OnlyCaseId })
    if ($cases.Count -ne 1) {
        throw "OnlyCaseId '$OnlyCaseId' did not select exactly one case."
    }
}
$manifest = [ordered]@{
    version = 1
    generated_at_utc = [DateTime]::UtcNow.ToString("o")
    scenario_set = $ScenarioSet
    controller_implementation = $ImplementationLabel
    single_run_script = $singleRun
    controllers = @($Controllers)
    additional_run_args = @($AdditionalRunArgs)
    expected_experiments = $cases.Count
    expected_controller_laps = $cases.Count * $Controllers.Count
    cases = @($cases)
}
$manifestTemp = "$ManifestPath.tmp"
$manifest | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $manifestTemp -Encoding utf8
Move-Item -Force -LiteralPath $manifestTemp -Destination $ManifestPath

$records = @()
if ($Resume -and (Test-Path -LiteralPath $statusPath -PathType Leaf)) {
    $records = @(Import-Csv -LiteralPath $statusPath)
    $validCaseIds = @{}
    foreach ($case in $cases) { $validCaseIds[$case.evaluation_case_id] = $true }
    $records = @($records | Where-Object { $validCaseIds.ContainsKey($_.evaluation_case_id) })
}

function Write-Status {
    param([object[]]$Rows)
    $fields = New-Object System.Collections.Generic.List[string]
    foreach ($row in $Rows) {
        foreach ($property in $row.PSObject.Properties.Name) {
            if (-not $fields.Contains($property)) { $fields.Add($property) }
        }
    }
    $temp = "$statusPath.tmp"
    if ($Rows.Count -eq 0) {
        "" | Set-Content -LiteralPath $temp -Encoding utf8
    }
    else {
        $Rows | Select-Object -Property @($fields) | Export-Csv -LiteralPath $temp -NoTypeInformation -Encoding utf8
    }
    Move-Item -Force -LiteralPath $temp -Destination $statusPath
}

function Test-CarlaRpcAvailable {
    param([int]$Port)
    Start-Sleep -Seconds 2
    $server = Get-Process -Name CarlaUE4, CarlaUE4-Win64-Shipping -ErrorAction SilentlyContinue
    if ($null -eq $server) { return $false }
    try {
        & $singleRun -HealthCheck -HealthPort $Port | Out-Null
        return $LASTEXITCODE -eq 0
    }
    catch { return $false }
}

function Test-CompletedCase {
    param([object]$Case, [object[]]$Rows)
    $caseRows = @($Rows | Where-Object { $_.evaluation_case_id -eq $Case.evaluation_case_id })
    if ($caseRows.Count -ne $Controllers.Count -or @($caseRows | Where-Object { $_.status -ne "ok" }).Count -gt 0) { return $false }
    $hashes = @($caseRows.trajectory_hash | Where-Object { -not [string]::IsNullOrWhiteSpace($_) } | Select-Object -Unique)
    if ($hashes.Count -ne 1) { return $false }
    $summaryPaths = @($caseRows.summary_path | Select-Object -Unique)
    return $summaryPaths.Count -eq 1 -and (Test-Path -LiteralPath $summaryPaths[0] -PathType Leaf)
}

function New-StatusRecord {
    param([object]$Case, [string]$Controller, [string]$Status, [string]$ErrorMessage, [string]$RunDir, [object]$SummaryRow)
    $data = [ordered]@{
        evaluation_case_id = $Case.evaluation_case_id
        scenario = $Case.scenario
        layer = $Case.layer
        requested_speed_kmh = $Case.speed_kmh
        requested_route_shape = $Case.route_shape
        requested_map = $Case.map_name
        seed = $Case.seed
        controller = $Controller
        status = $Status
        error = $ErrorMessage
        run_dir = $RunDir
        summary_path = if ($RunDir) { Join-Path $RunDir "summary.csv" } else { "" }
        trajectory_hash = ""
    }
    if ($null -ne $SummaryRow) {
        foreach ($property in $SummaryRow.PSObject.Properties) { $data[$property.Name] = $property.Value }
        $data["status"] = $Status
        $data["error"] = $ErrorMessage
        $data["run_dir"] = $RunDir
        $data["summary_path"] = Join-Path $RunDir "summary.csv"
    }
    return [pscustomobject]$data
}

foreach ($case in $cases) {
    if ($Resume -and (Test-CompletedCase -Case $case -Rows $records)) {
        Write-Host "Skipping validated completed case $($case.evaluation_case_id)"
        continue
    }
    $records = @($records | Where-Object { $_.evaluation_case_id -ne $case.evaluation_case_id })
    foreach ($controller in $Controllers) {
        $records += New-StatusRecord -Case $case -Controller $controller -Status "running" -ErrorMessage "" -RunDir "" -SummaryRow $null
    }
    Write-Status -Rows $records
    $started = Get-Date
    $provider = if ($null -ne $case.perturbation) { "noisy_ground_truth" } else { "ground_truth" }
    $runArgs = @(
        "--evaluation-headless",
        "--evaluation-case-id", $case.evaluation_case_id,
        "--controller-implementation", $ImplementationLabel,
        "--planner-mode", "legacy",
        "--planner-fallback", "error",
        "--target-speed", [string]$case.speed_kmh,
        "--route-min-length-m", [string]$case.route_length_m,
        "--route-shape", $case.route_shape,
        "--map-name", $case.map_name,
        "--seed", [string]$case.seed,
        "--error-provider", $provider,
        "--carla-port", [string]$RpcPort,
        "--controllers"
    ) + $Controllers

    if ($case.layer -eq "controller_only") {
        $runArgs = @("--speed-planner-mode", "off", "--speed-planner-limit-profile", "global") + $runArgs
    }
    else {
        $runArgs = @("--speed-planner-mode", "adaptive", "--speed-planner-limit-profile", "controller") + $runArgs
    }
    if ($null -ne $case.perturbation) {
        $p = $case.perturbation
        $runArgs = @(
            "--vehicle-mass-scale", [string]$p.vehicle_mass_scale,
            "--vehicle-moi-scale", [string]$p.vehicle_moi_scale,
            "--tire-friction-scale", [string]$p.tire_friction_scale,
            "--noise-lateral-std", [string]$p.noise_lateral_std,
            "--noise-heading-std-deg", [string]$p.noise_heading_std_deg,
            "--perception-delay-steps", [string]$p.perception_delay_steps,
            "--perception-dropout-probability", [string]$p.perception_dropout_probability,
            "--perception-smoothing-alpha", [string]$p.perception_smoothing_alpha
        ) + $runArgs
    }
    $runArgs = @($runArgs + $AdditionalRunArgs)

    $errorMessage = ""
    $stopForRpcLoss = $false
    try {
        & $singleRun @runArgs
        if ($LASTEXITCODE -ne 0) { throw "Controller run exited with code $LASTEXITCODE" }
        $pattern = "run_*_seed_$($case.seed)_$($case.route_shape)*"
        $candidates = Get-ChildItem -LiteralPath $logRoot -Directory |
            Where-Object { $_.LastWriteTime -ge $started.AddSeconds(-5) -and $_.Name -like $pattern } |
            Sort-Object LastWriteTime -Descending
        $runDir = $null
        $summaryPath = $null
        $summaryRows = @()
        foreach ($candidate in $candidates) {
            $candidateSummaryPath = Join-Path $candidate.FullName "summary.csv"
            if (-not (Test-Path -LiteralPath $candidateSummaryPath -PathType Leaf)) { continue }
            $candidateRows = @(Import-Csv -LiteralPath $candidateSummaryPath)
            $candidateControllers = (@($candidateRows.controller | Sort-Object -Unique) -join ",")
            $candidateCaseIds = @($candidateRows.evaluation_case_id | Sort-Object -Unique)
            if (
                $candidateRows.Count -eq $Controllers.Count -and
                $candidateControllers -eq $controllerSet -and
                $candidateCaseIds.Count -eq 1 -and
                $candidateCaseIds[0] -eq $case.evaluation_case_id
            ) {
                $runDir = $candidate
                $summaryPath = $candidateSummaryPath
                $summaryRows = $candidateRows
                break
            }
        }
        if ($null -eq $runDir) { throw "No controller/case-matched run directory matched $pattern" }
        $hashes = @($summaryRows.trajectory_hash | Select-Object -Unique)
        if ($hashes.Count -ne 1 -or [string]::IsNullOrWhiteSpace($hashes[0])) { throw "trajectory_hash mismatch in $summaryPath" }
        $records = @($records | Where-Object { $_.evaluation_case_id -ne $case.evaluation_case_id })
        foreach ($summary in $summaryRows) {
            $records += New-StatusRecord -Case $case -Controller $summary.controller -Status "ok" -ErrorMessage "" -RunDir $runDir.FullName -SummaryRow $summary
        }
    }
    catch {
        $errorMessage = $_.Exception.Message
        Write-Warning "$($case.evaluation_case_id): $errorMessage"
        $records = @($records | Where-Object { $_.evaluation_case_id -ne $case.evaluation_case_id })
        foreach ($controller in $Controllers) {
            $records += New-StatusRecord -Case $case -Controller $controller -Status "failed" -ErrorMessage $errorMessage -RunDir "" -SummaryRow $null
        }
        $stopForRpcLoss = -not (Test-CarlaRpcAvailable -Port $RpcPort)
    }
    Write-Status -Rows $records
    if ($stopForRpcLoss) {
        throw "CARLA RPC became unavailable on port $RpcPort; current failure was persisted for resumable recovery."
    }
}

if ($AllowControllerSubset) {
    # The standard aggregate/report contract intentionally requires PID/LQR/MPC
    # triplets. Preserve a complete raw artifact for targeted controller retests;
    # the later paired comparison merges it with the unchanged controllers.
    Copy-Item -Force -LiteralPath $statusPath -Destination (Join-Path $OutputRoot "raw_lap_results.csv")
}
else {
    & python $aggregateTool --manifest $ManifestPath --raw $statusPath --output-dir $OutputRoot
    if ($LASTEXITCODE -ne 0) { throw "Evaluation aggregation failed with exit code $LASTEXITCODE" }
    $reportPath = Join-Path $OutputRoot "controller_evaluation_report.html"
    & python $reportTool --input $OutputRoot --output $reportPath
    if ($LASTEXITCODE -ne 0) { throw "Evaluation report generation failed with exit code $LASTEXITCODE" }
}
Write-Host "Evaluation complete: $OutputRoot"
