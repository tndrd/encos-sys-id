"""Utilities for ENCOS motor position feedback.

The current CAN response decoder maps motor position into [-10, 10] rad.
Treat that value as wrapped feedback and unwrap it on the host when a
continuous multi-turn position is needed.
"""

from __future__ import annotations

import numpy as np

POS_MIN_RAD = -10.0
POS_MAX_RAD = 10.0
POS_PERIOD_RAD = POS_MAX_RAD - POS_MIN_RAD


def unwrap_motor_position(qpos: np.ndarray, period_rad: float = POS_PERIOD_RAD) -> np.ndarray:
    """Return continuous position from wrapped decoded motor position."""
    return np.unwrap(np.asarray(qpos, dtype=float), period=period_rad)


def rotation_check(
    qpos: np.ndarray,
    n_rotations: float,
    direction: int = 1,
    period_rad: float = POS_PERIOD_RAD,
) -> dict[str, float]:
    """Compare unwrapped position delta with expected full rotations."""
    if n_rotations == 0:
        raise ValueError("n_rotations must be non-zero")

    qpos_unwrapped = unwrap_motor_position(qpos, period_rad=period_rad)
    measured_delta_rad = float(qpos_unwrapped[-1] - qpos_unwrapped[0])
    expected_delta_rad = float(direction * n_rotations * 2 * np.pi)
    error_rad = measured_delta_rad - expected_delta_rad

    return {
        "measured_delta_rad": measured_delta_rad,
        "expected_delta_rad": expected_delta_rad,
        "error_rad": error_rad,
        "error_per_turn_rad": error_rad / n_rotations,
        "scale": measured_delta_rad / expected_delta_rad,
    }
