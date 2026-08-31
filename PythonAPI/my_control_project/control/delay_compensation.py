# uncompyle6 version 3.9.3
# Python bytecode version base 3.7.0 (3394)
# Decompiled from: Python 3.13.13 | packaged by Anaconda, Inc. | (main, Apr 14 2026, 06:12:50) [MSC v.1942 64 bit (AMD64)]
# Embedded file name: D:\WindowsNoEditor\PythonAPI\my_control_project\control\delay_compensation.py
# Compiled at: 2026-07-14 05:26:36
# Size of source mod 2**32: 595 bytes
import math

def _wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compensate_tracking_errors(tracking_errors, *, e_y_dot, e_psi_dot, delay_steps, dt):
    """Project delayed Frenet tracking errors to the current control instant."""
    result = dict(tracking_errors)
    delay_steps = int(delay_steps)
    if delay_steps <= 0:
        return result
    elapsed = delay_steps * float(dt)
    result["e_y"] = float(result["e_y"]) + elapsed * float(e_y_dot)
    result["e_psi"] = _wrap_angle(float(result["e_psi"]) + elapsed * float(e_psi_dot))
    return result
