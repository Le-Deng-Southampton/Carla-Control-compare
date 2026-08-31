"""Backward-compatible CARLA health-check import path.

The executable probe lives at the project root so PowerShell runners can call
it directly.  Keep this module for existing experiment imports.
"""

from carla_health import main, probe_carla_world

__all__ = ("main", "probe_carla_world")
