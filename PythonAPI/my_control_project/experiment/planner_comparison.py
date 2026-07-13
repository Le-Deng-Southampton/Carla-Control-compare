import argparse
import csv
import json


PAIR_KEY_FIELDS = (
    "requested_map",
    "seed",
    "requested_route_shape",
    "requested_speed_kmh",
    "speed_planner_mode",
    "error_provider",
    "controller",
)


def _number(row, name, default=0.0):
    value = row.get(name, default)
    if value in (None, ""):
        return float(default)
    return float(value)


def _boolean(value):
    return str(value).strip().lower() in ("1", "true", "yes", "passed", "ok")


def _improvement_percent(legacy, frenet):
    if abs(legacy) < 1e-12:
        return 0.0 if abs(frenet) < 1e-12 else float("-inf")
    return (legacy - frenet) * 100.0 / legacy


def _pair_rows(key, legacy, frenet):
    legacy_collision = _number(legacy, "collision_count")
    frenet_collision = _number(frenet, "collision_count")
    legacy_violations = _number(legacy, "lane_boundary_violation_count")
    frenet_violations = _number(frenet, "lane_boundary_violation_count")
    legacy_clearance = _number(legacy, "min_lane_clearance_m")
    frenet_clearance = _number(frenet, "min_lane_clearance_m")
    reasons = []
    if frenet_collision > legacy_collision:
        reasons.append("collision_count")
    if frenet_violations > legacy_violations:
        reasons.append("lane_boundary_violation_count")
    if frenet_clearance < legacy_clearance - 1e-9:
        reasons.append("min_lane_clearance_m")
    if legacy.get("status", "ok") == "ok" and frenet.get("status", "ok") != "ok":
        reasons.append("route_generation_status")

    result = {field: key[index] for index, field in enumerate(PAIR_KEY_FIELDS)}
    result.update({
        "legacy_trajectory_hash": legacy["trajectory_hash"],
        "frenet_trajectory_hash": frenet["trajectory_hash"],
        "legacy_status": legacy.get("status", "ok"),
        "frenet_status": frenet.get("status", "ok"),
        "route_generation_status": f"legacy:{legacy.get('status', 'ok')};frenet:{frenet.get('status', 'ok')}",
        "stability_delta": int(_boolean(frenet.get("stability_passed"))) - int(_boolean(legacy.get("stability_passed"))),
        "rms_e_y_improvement_pct": _improvement_percent(
            _number(legacy, "rms_e_y"), _number(frenet, "rms_e_y")
        ),
        "p95_steer_delta_change": _number(frenet, "p95_abs_steer_delta") - _number(legacy, "p95_abs_steer_delta"),
        "steering_reversal_change": (
            _number(frenet, "significant_steer_reversals_per_10s")
            - _number(legacy, "significant_steer_reversals_per_10s")
        ),
        "route_completion_change_pct": _number(frenet, "route_completion_pct") - _number(legacy, "route_completion_pct"),
        "hard_safety_regression": bool(reasons),
        "hard_safety_reasons": json.dumps(reasons, separators=(",", ":")),
    })
    return result


def pair_planner_rows(rows):
    groups = {}
    for row in rows:
        mode = str(row.get("planner_mode", "")).strip().lower()
        if mode not in ("legacy", "frenet"):
            raise ValueError("planner_mode must be legacy or frenet")
        if not str(row.get("trajectory_hash", "")).strip():
            raise ValueError("trajectory_hash is required for every planner row")
        key = tuple(str(row.get(field, "")) for field in PAIR_KEY_FIELDS)
        bucket = groups.setdefault(key, {})
        if mode in bucket:
            raise ValueError(f"duplicate {mode} row for paired scenario {key}")
        bucket[mode] = row

    paired = []
    for key in sorted(groups):
        bucket = groups[key]
        if set(bucket) != {"legacy", "frenet"}:
            raise ValueError(f"complete legacy/frenet pair required for scenario {key}")
        paired.append(_pair_rows(key, bucket["legacy"], bucket["frenet"]))
    return paired


def main(argv=None):
    parser = argparse.ArgumentParser(description="Pair legacy and Frenet matrix rows.")
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    with open(args.input, newline="", encoding="utf-8-sig") as handle:
        paired = pair_planner_rows(list(csv.DictReader(handle)))
    fieldnames = list(paired[0]) if paired else []
    with open(args.output, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        if fieldnames:
            writer.writeheader()
            writer.writerows(paired)
    return 1 if any(row["hard_safety_regression"] for row in paired) else 0


if __name__ == "__main__":
    raise SystemExit(main())
