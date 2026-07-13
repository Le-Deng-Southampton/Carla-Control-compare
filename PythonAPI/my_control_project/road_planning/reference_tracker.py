from dataclasses import dataclass, replace
import math

import numpy as np


def _normalize_angle(angle_rad):
    return (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi


class ReferenceTrackingFailure(RuntimeError):
    """Raised after the tracker cannot recover a valid route projection."""


@dataclass(frozen=True)
class ProjectionResult:
    s_ref_m: float
    reference_sample: object
    segment_index: int
    projection_distance_m: float
    lateral_error_m: float
    heading_error_rad: float
    progress_delta_m: float
    state: str = "normal"
    held: bool = False


class ReferenceTracker:
    """Track a frozen reference trajectory using continuous arc-length projection."""

    def __init__(self, trajectory, max_rollback_m=2.0, hold_steps=3):
        if max_rollback_m < 0.0:
            raise ValueError("max_rollback_m must be non-negative")
        if hold_steps < 0:
            raise ValueError("hold_steps must be non-negative")
        self.trajectory = trajectory
        self.max_rollback_m = float(max_rollback_m)
        self.hold_steps = int(hold_steps)
        self._last_result = None
        self._failure_count = 0

        self._segment_x = np.asarray(trajectory.x_m[1:] - trajectory.x_m[:-1], dtype=float)
        self._segment_y = np.asarray(trajectory.y_m[1:] - trajectory.y_m[:-1], dtype=float)
        self._segment_length_sq = self._segment_x ** 2 + self._segment_y ** 2
        if np.any(self._segment_length_sq <= 1e-12):
            raise ValueError("Reference trajectory contains a zero-length segment")
        self._segment_length = np.sqrt(self._segment_length_sq)
        self._segment_yaw = np.arctan2(self._segment_y, self._segment_x)

    @property
    def current_s_m(self):
        if self._last_result is None:
            return float(self.trajectory.s_m[0])
        return float(self._last_result.s_ref_m)

    def _candidate_indices(self, speed_mps, allow_recovery):
        count = len(self._segment_x)
        if self._last_result is None:
            return np.arange(count, dtype=int)

        previous_s = self._last_result.s_ref_m
        rollback = 10.0 if allow_recovery else self.max_rollback_m
        forward = 200.0 if allow_recovery else max(120.0, 3.0 * abs(float(speed_mps)))
        starts = self.trajectory.s_m[:-1]
        ends = self.trajectory.s_m[1:]
        return np.nonzero(
            (ends >= previous_s - rollback) & (starts <= previous_s + forward)
        )[0]

    def _project(self, x_m, y_m, yaw_rad, speed_mps, allow_recovery):
        indices = self._candidate_indices(speed_mps, allow_recovery)
        if len(indices) == 0:
            return None

        rel_x = float(x_m) - self.trajectory.x_m[:-1][indices]
        rel_y = float(y_m) - self.trajectory.y_m[:-1][indices]
        ratio = np.clip(
            (rel_x * self._segment_x[indices] + rel_y * self._segment_y[indices])
            / self._segment_length_sq[indices],
            0.0,
            1.0,
        )
        projected_x = self.trajectory.x_m[:-1][indices] + ratio * self._segment_x[indices]
        projected_y = self.trajectory.y_m[:-1][indices] + ratio * self._segment_y[indices]
        error_x = float(x_m) - projected_x
        error_y = float(y_m) - projected_y
        distance = np.hypot(error_x, error_y)
        heading_error = np.asarray(
            [_normalize_angle(float(yaw_rad) - self._segment_yaw[index]) for index in indices]
        )
        candidate_s = (
            self.trajectory.s_m[:-1][indices] + ratio * self._segment_length[indices]
        )

        score = distance + 2.0 * np.abs(heading_error)
        if self._last_result is not None:
            progress_delta = candidate_s - self._last_result.s_ref_m
            score += 0.02 * np.maximum(-progress_delta, 0.0)

        best_position = min(
            range(len(indices)),
            key=lambda position: (float(score[position]), int(indices[position])),
        )
        segment_index = int(indices[best_position])
        s_ref = float(candidate_s[best_position])
        sample = self.trajectory.sample_at(s_ref)
        tangent_x = self._segment_x[segment_index] / self._segment_length[segment_index]
        tangent_y = self._segment_y[segment_index] / self._segment_length[segment_index]
        lateral_error = tangent_x * float(error_y[best_position]) - tangent_y * float(error_x[best_position])
        previous_s = self._last_result.s_ref_m if self._last_result is not None else s_ref
        return ProjectionResult(
            s_ref_m=s_ref,
            reference_sample=sample,
            segment_index=segment_index,
            projection_distance_m=float(distance[best_position]),
            lateral_error_m=float(lateral_error),
            heading_error_rad=float(heading_error[best_position]),
            progress_delta_m=float(s_ref - previous_s),
        )

    def _hold_or_fail(self, reason):
        self._failure_count += 1
        if self._last_result is None or self._failure_count > self.hold_steps:
            raise ReferenceTrackingFailure(reason)
        return replace(
            self._last_result,
            progress_delta_m=0.0,
            state="held_" + reason,
            held=True,
        )

    def update(self, x_m, y_m, yaw_rad, speed_mps, control_dt, allow_recovery=False):
        projection = self._project(x_m, y_m, yaw_rad, speed_mps, allow_recovery)
        if projection is None:
            return self._hold_or_fail("no_projection")

        clearance = max(
            projection.reference_sample.left_clearance_m,
            projection.reference_sample.right_clearance_m,
        )
        if projection.projection_distance_m > clearance + 1.0:
            return self._hold_or_fail("off_route")

        if self._last_result is not None:
            delta = projection.s_ref_m - self._last_result.s_ref_m
            if delta < 0.0:
                if allow_recovery and delta >= -self.max_rollback_m:
                    projection = replace(projection, state="rollback")
                elif allow_recovery:
                    return self._hold_or_fail("rollback_limit")
                else:
                    self._failure_count = 0
                    return replace(
                        self._last_result,
                        progress_delta_m=0.0,
                        state="normal",
                        held=False,
                    )

            maximum_jump = max(5.0, 3.0 * abs(float(speed_mps)) * max(float(control_dt), 0.0))
            if delta > maximum_jump:
                return self._hold_or_fail("progress_jump")

        self._failure_count = 0
        self._last_result = projection
        return projection

    def target_sample(self, lookahead_m):
        return self.trajectory.sample_at(self.current_s_m + max(0.0, float(lookahead_m)))

    def preview_s_m(self, distances_m):
        return [
            float(np.clip(self.current_s_m + float(distance), 0.0, self.trajectory.length_m))
            for distance in distances_m
        ]

    def curvature_preview(self, distances_m):
        return [self.trajectory.sample_at(s_m).curvature_1pm for s_m in self.preview_s_m(distances_m)]

    def is_complete(self, x_m, y_m, remaining_tolerance_m=2.0, endpoint_tolerance_m=8.0):
        remaining = self.trajectory.length_m - self.current_s_m
        endpoint_dx = float(x_m) - float(self.trajectory.x_m[-1])
        endpoint_dy = float(y_m) - float(self.trajectory.y_m[-1])
        endpoint_distance = math.hypot(endpoint_dx, endpoint_dy)
        return remaining <= float(remaining_tolerance_m) and endpoint_distance <= float(endpoint_tolerance_m)
