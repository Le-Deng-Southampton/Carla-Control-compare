# uncompyle6 version 3.9.3
# Python bytecode version base 3.7.0 (3394)
# Decompiled from: Python 3.13.13 | packaged by Anaconda, Inc. | (main, Apr 14 2026, 06:12:50) [MSC v.1942 64 bit (AMD64)]
# Embedded file name: D:\WindowsNoEditor\PythonAPI\my_control_project\control\common.py
# Compiled at: 2026-06-29 02:46:46
# Size of source mod 2**32: 2888 bytes
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class BicycleModelParams:
    front_length = 1.45
    front_length: float
    rear_length = 1.45
    rear_length: float
    mass = 1750.0
    mass: float
    yaw_inertia = 2875.0
    yaw_inertia: float
    front_cornering_stiffness = 19000.0
    front_cornering_stiffness: float
    rear_cornering_stiffness = 33000.0
    rear_cornering_stiffness: float

    @property
    def wheelbase(self):
        return self.front_length + self.rear_length

    def cornering_stiffness(self, scale=1.0):
        return (
         self.front_cornering_stiffness * scale, self.rear_cornering_stiffness * scale)


DEFAULT_BICYCLE_MODEL = BicycleModelParams()

def error_progress(value, start, full):
    start = max(float(start), 0.0)
    full = max(float(full), start + 1e-06)
    return float(np.clip((abs(float(value)) - start) / (full - start), 0.0, 1.0))


def select_speed_profile(speed_mps, base_profile_factory, speed_profiles, enabled=True):
    return enabled and speed_profiles or base_profile_factory()
    speed_kmh = float(speed_mps) * 3.6
    for profile in speed_profiles:
        if speed_kmh < profile["max_speed_kmh"]:
            return profile

    return speed_profiles[-1]


def preview_anticipation_blend(current_curvature, preview_curvature, blend, tracking_errors, *, enabled=True, preview_delta, max_blend):
    if enabled:
        return tracking_errors or blend
    else:
        near_centerline = abs(tracking_errors.get("e_y", 0.0)) <= 0.75 and abs(tracking_errors.get("e_psi", 0.0)) <= np.radians(7.0)
        return near_centerline or blend
    if abs(preview_curvature) <= abs(current_curvature) + preview_delta:
        return blend
    entry_progress = error_progress(preview_curvature, 0.02, 0.08)
    return float(np.clip(blend + 0.26 * entry_progress, blend, max_blend))


def resolve_preview_curvature(curvature, *, blend, tracking_errors=None, scheduling_enabled=True, preview_weight_end, preview_delta, max_blend):
    values = np.asarray(curvature, dtype=float).reshape(-1)
    if values.size == 0:
        return (0.0, 0.0)
    current_curvature = float(values[0])
    if values.size == 1:
        return (
         current_curvature, current_curvature)
    weights = np.linspace(1.0, preview_weight_end, values.size)
    preview_index = int(np.argmax(np.abs(values) * weights))
    preview_curvature = float(values[preview_index])
    resolved_blend = float(np.clip(blend, 0.0, 1.0))
    resolved_blend = preview_anticipation_blend(current_curvature,
      preview_curvature,
      resolved_blend,
      tracking_errors,
      enabled=scheduling_enabled,
      preview_delta=preview_delta,
      max_blend=max_blend)
    feedforward_curvature = (1.0 - resolved_blend) * current_curvature + resolved_blend * preview_curvature
    return (current_curvature, feedforward_curvature)
