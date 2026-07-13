# PID Control Desensitization Design

## Goal

Remove only the PID runtime adaptations that existing CARLA logs directly associate with excessive steering sensitivity, while preserving the controller interface, curvature feedforward, integral protection, and the LQR/MPC implementations.

## Evidence

The Town04 fixed-speed run at 30 km/h (`seed=457235040`, `spawn_index=13`) kept PID in `pid_crawl_recovery` for all 1,830 samples. PID recorded a mean absolute steering delta of `0.0308067331`, a maximum delta of `0.2074486427`, 291 samples at or above approximately `0.10`, and 189 significant steering sign reversals. In the same run, LQR recorded no significant sign reversals and MPC remained bounded by its `0.06` rate limiter.

The PID implementation can expand a configured `0.10` rate limit through the low-speed profile and curvature multiplier, while also raising proportional and derivative gains at runtime. These mechanisms are the evidence-backed removal target. Speed-profile transitions are not the primary cause because the low-speed run never changed profile.

## Design

- Keep `PidControllerAdapter`, `run_step()`, all current CLI wiring, and the `speed_scheduling_enabled` constructor argument/attribute for compatibility.
- Delete PID speed-profile tables and runtime lateral gain changes. The CARLA lateral PID keeps the gains supplied at construction.
- Use one non-negative per-step steering limit: `max(float(self.max_steer_rate), 0.0)`.
- Keep curvature preview and feedforward, but use only the configured `curvature_preview_blend` and `curvature_feedforward_gain`; speed and tracking error no longer change them.
- Keep integral separation, integral clipping, steering saturation, saturation-triggered integral clearing, and zero steering state on reset.
- Keep diagnostics compatible: `last_speed_profile` is always `pid_base` and `last_curvature_feedforward_scale` is always `1.0`.
- Do not edit LQR, MPC, shared control helpers, controller factory, or CLI configuration.

## Verification

Use test-first changes to prove that PID no longer retunes gains, never exceeds the configured steering-rate limit, uses a fixed preview ratio, and reports `pid_base`. Preserve the existing feedforward and integral-protection tests. Run the full unit suite, then repeat the recorded Town04 scenario and compare steering and tracking metrics with the saved baseline.

If CARLA cannot run, report unit-level verification only and do not claim that the simulator-level oscillation acceptance criteria passed.
