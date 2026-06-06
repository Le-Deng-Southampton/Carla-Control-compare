class CollisionZeroSpeedTerminator:
    """Detects a collision followed by sustained near-zero vehicle speed."""

    def __init__(self, timeout_seconds=2.0, speed_threshold_mps=0.1):
        self.timeout_seconds = timeout_seconds
        self.speed_threshold_mps = speed_threshold_mps
        self._has_collision = False
        self._zero_speed_started_at = None

    def update(self, speed_mps, sim_time, collision_count):
        if collision_count > 0:
            self._has_collision = True

        if not self._has_collision:
            return False

        if speed_mps > self.speed_threshold_mps:
            self._zero_speed_started_at = None
            return False

        if self._zero_speed_started_at is None:
            self._zero_speed_started_at = sim_time
            return False

        return sim_time - self._zero_speed_started_at >= self.timeout_seconds

    def reset(self):
        self._has_collision = False
        self._zero_speed_started_at = None
