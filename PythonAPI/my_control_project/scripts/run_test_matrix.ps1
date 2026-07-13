param(
    [ValidateSet("smoke", "extended", "stability")]
    [string]$ScenarioSet = "smoke",

    [ValidateSet("off", "adaptive")]
    [string]$SpeedPlannerMode = "adaptive",

    [ValidateSet("legacy", "frenet", "both")]
    [string]$PlannerMode = "both",

    [ValidateSet("ground_truth", "noisy_ground_truth", "perception_proxy")]
    [string]$ErrorProvider = "ground_truth",

    [string[]]$Controllers = @("lqr", "pid", "mpc"),

    [int]$RpcPort = 2000,

    [int]$CarlaStartupTimeoutSeconds = 120,

    [switch]$ContinueOnFailure
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectRoot = Split-Path -Parent $scriptDir
$runner = Join-Path $scriptDir "run_my_control.ps1"
$logRoot = Join-Path $projectRoot "log"
$carlaRoot = Split-Path -Parent (Split-Path -Parent $projectRoot)
$carlaLauncher = Join-Path $carlaRoot "CarlaUE4.exe"
if (-not (Test-Path -LiteralPath $carlaLauncher -PathType Leaf)) {
    $carlaLauncher = Join-Path $carlaRoot "CarlaUE4\Binaries\Win64\CarlaUE4-Win64-Shipping.exe"
}

if (-not (Test-Path -LiteralPath $runner -PathType Leaf)) {
    throw "Single-run controller launcher was not found: $runner"
}

$scenarioCatalog = @{
    smoke = @(
        @{ Name = "low_network_curves"; Speed = 50; Route = "straight"; Map = "Town04"; Seed = 26050101 },
        @{ Name = "mid_s_curve"; Speed = 70; Route = "s_curve"; Map = "Town03"; Seed = 26070102 },
        @{ Name = "high_network_straight"; Speed = 120; Route = "straight"; Map = "Town04"; Seed = 26120103 }
    )
    extended = @(
        @{ Name = "low_network_curves"; Speed = 50; Route = "straight"; Map = "Town04"; Seed = 26050101 },
        @{ Name = "low_s_curve"; Speed = 50; Route = "s_curve"; Map = "Town03"; Seed = 26050102 },
        @{ Name = "low_curvy"; Speed = 50; Route = "curvy"; Map = "Town05"; Seed = 26050103 },
        @{ Name = "mid_straight"; Speed = 70; Route = "straight"; Map = "Town04"; Seed = 26070101 },
        @{ Name = "mid_s_curve"; Speed = 70; Route = "s_curve"; Map = "Town03"; Seed = 26070102 },
        @{ Name = "mid_curvy"; Speed = 70; Route = "curvy"; Map = "Town05"; Seed = 26070103 },
        @{ Name = "high_network_straight"; Speed = 120; Route = "straight"; Map = "Town04"; Seed = 26120103 },
        @{ Name = "high_gentle_curve"; Speed = 120; Route = "gentle_curve"; Map = "Town10HD"; Seed = 26120104 },
        @{ Name = "high_s_curve"; Speed = 120; Route = "s_curve"; Map = "Town05"; Seed = 26120105 }
    )
    stability = @(
        @{ Name = "urban_30_true_straight"; Speed = 30; Route = "true_straight"; Map = "Town04"; Seed = 26030101 },
        @{ Name = "urban_30_curvy"; Speed = 30; Route = "curvy"; Map = "Town05"; Seed = 26030102 },
        @{ Name = "urban_50_true_straight"; Speed = 50; Route = "true_straight"; Map = "Town04"; Seed = 26050101 },
        @{ Name = "urban_50_s_curve"; Speed = 50; Route = "s_curve"; Map = "Town03"; Seed = 26050102 },
        @{ Name = "urban_50_curvy"; Speed = 50; Route = "curvy"; Map = "Town05"; Seed = 26050103 },
        @{ Name = "road_70_true_straight"; Speed = 70; Route = "true_straight"; Map = "Town04"; Seed = 26070101 },
        @{ Name = "road_70_s_curve"; Speed = 70; Route = "s_curve"; Map = "Town03"; Seed = 26070102 },
        @{ Name = "road_70_curvy"; Speed = 70; Route = "curvy"; Map = "Town05"; Seed = 26070103 },
        @{ Name = "highway_120_true_straight"; Speed = 120; Route = "true_straight"; Map = "Town04"; Seed = 26120103 },
        @{ Name = "highway_120_gentle_curve"; Speed = 120; Route = "gentle_curve"; Map = "Town10HD"; Seed = 26120104 }
    )
}

function Test-CarlaRpc {
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

function Ensure-Carla {
    param(
        [string]$Launcher,
        [int]$Port,
        [int]$TimeoutSeconds
    )

    if (-not (Test-Path -LiteralPath $Launcher -PathType Leaf)) {
        throw "CARLA launcher was not found: $Launcher"
    }

    if (-not (Test-CarlaRpc -Port $Port)) {
        $processes = Get-Process | Where-Object { $_.ProcessName -in @("CarlaUE4", "CarlaUE4-Win64-Shipping") }
        if (-not $processes) {
            Write-Host "Starting CARLA simulator on RPC port $Port..."
            Start-Process -FilePath $Launcher -ArgumentList "-carla-rpc-port=$Port", "-quality-level=Low" -WorkingDirectory $carlaRoot -WindowStyle Hidden
        }
    }

    $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
    while ((Get-Date) -lt $deadline) {
        if (Test-CarlaRpc -Port $Port) {
            return
        }
        Start-Sleep -Seconds 2
    }

    throw "CARLA RPC port $Port was not reachable within $TimeoutSeconds seconds."
}

$scenarios = $scenarioCatalog[$ScenarioSet]
$testCases = @()
if ($ScenarioSet -eq "stability") {
    foreach ($scenario in $scenarios) {
        foreach ($mode in @("off", "adaptive")) {
            $testCases += [pscustomobject]@{
                Scenario = $scenario
                Mode = $mode
                Provider = "ground_truth"
            }
        }
    }
    foreach ($scenarioName in @("urban_30_curvy", "road_70_s_curve", "highway_120_gentle_curve")) {
        $scenario = $scenarios | Where-Object { $_.Name -eq $scenarioName } | Select-Object -First 1
        $testCases += [pscustomobject]@{
            Scenario = $scenario
            Mode = "adaptive"
            Provider = "perception_proxy"
        }
    }
}
else {
    foreach ($scenario in $scenarios) {
        $testCases += [pscustomobject]@{
            Scenario = $scenario
            Mode = $SpeedPlannerMode
            Provider = $ErrorProvider
        }
    }
}
$plannerModes = if ($PlannerMode -eq "both") { @("legacy", "frenet") } else { @($PlannerMode) }
$expandedTestCases = @()
foreach ($testCase in $testCases) {
    foreach ($casePlannerMode in $plannerModes) {
        $expandedTestCases += [pscustomobject]@{
            Scenario = $testCase.Scenario
            Mode = $testCase.Mode
            Provider = $testCase.Provider
            PlannerMode = $casePlannerMode
        }
    }
}
$testCases = $expandedTestCases
$matrixTimestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$matrixSummaryPath = Join-Path $logRoot "matrix_${matrixTimestamp}_${ScenarioSet}.csv"
$pairedSummaryPath = $matrixSummaryPath -replace "\.csv$", "_paired.csv"
$plannerComparison = Join-Path $projectRoot "experiment\planner_comparison.py"
$records = New-Object System.Collections.Generic.List[object]

function New-MatrixRecord {
    param(
        [string]$Status,
        [string]$ErrorMessage,
        [object]$Scenario,
        [string]$Mode,
        [string]$Provider,
        [string]$PlannerMode,
        [string]$RunDirectory = "",
        [object]$SummaryRow = $null
    )

    return [pscustomobject][ordered]@{
        status = $Status
        error = $ErrorMessage
        scenario = $Scenario.Name
        speed_planner_mode = $Mode
        planner_mode = $PlannerMode
        error_provider = $Provider
        requested_speed_kmh = $Scenario.Speed
        requested_route_shape = $Scenario.Route
        requested_map = $Scenario.Map
        seed = $Scenario.Seed
        run_dir = $RunDirectory
        controller = if ($null -ne $SummaryRow) { $SummaryRow.controller } else { "" }
        trajectory_hash = if ($null -ne $SummaryRow) { $SummaryRow.trajectory_hash } else { "" }
        route_label = if ($null -ne $SummaryRow) { $SummaryRow.route_label } else { "" }
        target_speed_kmh = if ($null -ne $SummaryRow) { $SummaryRow.target_speed_kmh } else { "" }
        mean_abs_e_y = if ($null -ne $SummaryRow) { $SummaryRow.mean_abs_e_y } else { "" }
        rms_e_y = if ($null -ne $SummaryRow) { $SummaryRow.rms_e_y } else { "" }
        max_abs_e_y = if ($null -ne $SummaryRow) { $SummaryRow.max_abs_e_y } else { "" }
        mean_abs_e_psi_deg = if ($null -ne $SummaryRow) { $SummaryRow.mean_abs_e_psi_deg } else { "" }
        mean_abs_speed_error = if ($null -ne $SummaryRow) { $SummaryRow.mean_abs_speed_error } else { "" }
        mean_abs_steer_delta = if ($null -ne $SummaryRow) { $SummaryRow.mean_abs_steer_delta } else { "" }
        max_abs_steer_delta = if ($null -ne $SummaryRow) { $SummaryRow.max_abs_steer_delta } else { "" }
        p95_abs_steer_delta = if ($null -ne $SummaryRow) { $SummaryRow.p95_abs_steer_delta } else { "" }
        significant_steer_reversals_per_10s = if ($null -ne $SummaryRow) { $SummaryRow.significant_steer_reversals_per_10s } else { "" }
        max_significant_steer_reversals_2s = if ($null -ne $SummaryRow) { $SummaryRow.max_significant_steer_reversals_2s } else { "" }
        steer_rate_limit_hit_pct = if ($null -ne $SummaryRow) { $SummaryRow.steer_rate_limit_hit_pct } else { "" }
        p95_abs_speed_error = if ($null -ne $SummaryRow) { $SummaryRow.p95_abs_speed_error } else { "" }
        p95_abs_lateral_accel = if ($null -ne $SummaryRow) { $SummaryRow.p95_abs_lateral_accel } else { "" }
        p95_abs_lateral_jerk = if ($null -ne $SummaryRow) { $SummaryRow.p95_abs_lateral_jerk } else { "" }
        p95_abs_longitudinal_jerk = if ($null -ne $SummaryRow) { $SummaryRow.p95_abs_longitudinal_jerk } else { "" }
        throttle_brake_switches_per_10s = if ($null -ne $SummaryRow) { $SummaryRow.throttle_brake_switches_per_10s } else { "" }
        route_completion_pct = if ($null -ne $SummaryRow) { $SummaryRow.route_completion_pct } else { "" }
        collision_count = if ($null -ne $SummaryRow) { $SummaryRow.collision_count } else { "" }
        lane_boundary_violation_count = if ($null -ne $SummaryRow) { $SummaryRow.lane_boundary_violation_count } else { "" }
        min_lane_clearance_m = if ($null -ne $SummaryRow) { $SummaryRow.min_lane_clearance_m } else { "" }
        max_abs_lateral_accel = if ($null -ne $SummaryRow) { $SummaryRow.max_abs_lateral_accel } else { "" }
        min_planned_speed_kmh = if ($null -ne $SummaryRow) { $SummaryRow.min_planned_speed_kmh } else { "" }
        mean_planned_speed_kmh = if ($null -ne $SummaryRow) { $SummaryRow.mean_planned_speed_kmh } else { "" }
        stability_passed = if ($null -ne $SummaryRow) { $SummaryRow.stability_passed } else { "" }
        stability_fail_reasons = if ($null -ne $SummaryRow) { $SummaryRow.stability_fail_reasons } else { "" }
        stability_fail_windows = if ($null -ne $SummaryRow) { $SummaryRow.stability_fail_windows } else { "" }
    }
}

Write-Host "Controller test matrix"
Write-Host "  Scenario set:       $ScenarioSet"
Write-Host "  Speed planner mode: $SpeedPlannerMode"
Write-Host "  Planner mode:       $PlannerMode"
Write-Host "  Error provider:     $ErrorProvider"
Write-Host "  Controllers:        $($Controllers -join ', ')"
Write-Host "  Matrix summary:     $matrixSummaryPath"
Write-Host ""

foreach ($testCase in $testCases) {
    $scenario = $testCase.Scenario
    $caseMode = $testCase.Mode
    $caseProvider = $testCase.Provider
    $casePlannerMode = $testCase.PlannerMode
    $startTime = Get-Date
    $scenarioError = ""
    $scenarioSucceeded = $false
    Write-Host "Starting scenario '$($scenario.Name)' planner=$casePlannerMode mode=$caseMode provider=$caseProvider speed=$($scenario.Speed) route=$($scenario.Route) map=$($scenario.Map) seed=$($scenario.Seed)"

    $runArgs = @(
        "--speed-planner-mode", $caseMode,
        "--planner-mode", $casePlannerMode,
        "--speed-planner-limit-profile", "global",
        "--target-speed", [string]$scenario.Speed,
        "--route-shape", $scenario.Route,
        "--map-name", $scenario.Map,
        "--seed", [string]$scenario.Seed,
        "--error-provider", $caseProvider,
        "--controllers"
    ) + $Controllers

    try {
        Ensure-Carla -Launcher $carlaLauncher -Port $RpcPort -TimeoutSeconds $CarlaStartupTimeoutSeconds
        & $runner @runArgs
        if ($LASTEXITCODE -ne 0) {
            throw "Scenario '$($scenario.Name)' failed with exit code $LASTEXITCODE"
        }
        $scenarioSucceeded = $true
    }
    catch {
        $scenarioError = $_.Exception.Message
        Write-Warning $scenarioError
        if (-not $ContinueOnFailure -and $ScenarioSet -ne "stability") {
            throw
        }
    }

    if (-not $scenarioSucceeded) {
        $records.Add((New-MatrixRecord -Status "failed" -ErrorMessage $scenarioError -Scenario $scenario -Mode $caseMode -Provider $caseProvider -PlannerMode $casePlannerMode))
        $records | Export-Csv -LiteralPath $matrixSummaryPath -NoTypeInformation
        Write-Host "Finished scenario '$($scenario.Name)' with failure."
        Write-Host ""
        continue
    }

    $expectedRunName = "run_*_seed_$($scenario.Seed)_$($scenario.Route)"
    $runDir = Get-ChildItem -Path $logRoot -Directory |
        Where-Object { $_.LastWriteTime -ge $startTime.AddSeconds(-5) -and $_.Name -like $expectedRunName } |
        Sort-Object LastWriteTime -Descending |
        Select-Object -First 1

    if (-not $runDir) {
        Write-Warning "No matching run directory found for scenario '$($scenario.Name)' using pattern '$expectedRunName'."
        $records.Add((New-MatrixRecord -Status "missing_run" -ErrorMessage "No matching run directory found for pattern '$expectedRunName'." -Scenario $scenario -Mode $caseMode -Provider $caseProvider -PlannerMode $casePlannerMode))
        $records | Export-Csv -LiteralPath $matrixSummaryPath -NoTypeInformation
        continue
    }

    $summaryPath = Join-Path $runDir.FullName "summary.csv"
    if (-not (Test-Path -LiteralPath $summaryPath -PathType Leaf)) {
        Write-Warning "summary.csv was not found for scenario '$($scenario.Name)': $summaryPath"
        $records.Add((New-MatrixRecord -Status "missing_summary" -ErrorMessage "summary.csv was not found: $summaryPath" -Scenario $scenario -Mode $caseMode -Provider $caseProvider -PlannerMode $casePlannerMode -RunDirectory $runDir.FullName))
        $records | Export-Csv -LiteralPath $matrixSummaryPath -NoTypeInformation
        continue
    }

    foreach ($row in (Import-Csv -LiteralPath $summaryPath)) {
        $stabilityPassed = [string]$row.stability_passed -eq "True"
        $status = if ($stabilityPassed) { "ok" } else { "stability_failed" }
        $errorMessage = if ($stabilityPassed) { "" } else { $row.stability_fail_reasons }
        $records.Add((New-MatrixRecord -Status $status -ErrorMessage $errorMessage -Scenario $scenario -Mode $caseMode -Provider $caseProvider -PlannerMode $casePlannerMode -RunDirectory $runDir.FullName -SummaryRow $row))
    }

    $records | Export-Csv -LiteralPath $matrixSummaryPath -NoTypeInformation
    Write-Host "Finished scenario '$($scenario.Name)'."
    Write-Host ""
}

$records | Export-Csv -LiteralPath $matrixSummaryPath -NoTypeInformation
Write-Host "Matrix complete: $matrixSummaryPath"

$pairedFailed = $false
if ($PlannerMode -eq "both") {
    & python $plannerComparison --input $matrixSummaryPath --output $pairedSummaryPath
    if ($LASTEXITCODE -ne 0) {
        $pairedFailed = $true
    }
    if (Test-Path -LiteralPath $pairedSummaryPath -PathType Leaf) {
        $hardSafetyRegressions = @(
            Import-Csv -LiteralPath $pairedSummaryPath |
                Where-Object { [string]$_.hard_safety_regression -eq "True" }
        )
        if ($hardSafetyRegressions.Count -gt 0) {
            $pairedFailed = $true
        }
    }
    Write-Host "Paired planner summary: $pairedSummaryPath"
}

$failedRecords = @($records | Where-Object { $_.status -ne "ok" })
if ($failedRecords.Count -gt 0 -or $pairedFailed) {
    Write-Error "$($failedRecords.Count) matrix result(s) failed stability or execution requirements. See $matrixSummaryPath"
    exit 1
}
