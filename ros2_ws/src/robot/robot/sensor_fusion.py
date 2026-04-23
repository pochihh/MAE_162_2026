"""
sensor_fusion.py — pluggable sensor fusion strategies
======================================================
Orientation strategies share this interface::

    fused_theta_rad = strategy.update(odom_theta, mag_heading, linear_vel, angular_vel)

Position strategies share this interface::

    fused_x_mm, fused_y_mm = strategy.update(odom_x, odom_y, gps_x, gps_y)

Usage in robot.py / main.py
----------------------------
    from robot.sensor_fusion import AdaptiveComplementaryFilter
    robot.set_orientation_fusion_strategy(AdaptiveComplementaryFilter())
"""

from __future__ import annotations

import math


def _wrap(angle: float) -> float:
    """Wrap angle difference to [-π, π]."""
    return math.atan2(math.sin(angle), math.cos(angle))


# =============================================================================
# Abstract base
# =============================================================================

class SensorFusion:
    """Abstract base for all sensor fusion strategies."""
    def __init__(self):
        self.measurement_type = None  # "orientation" or "position"

    def update(self, *args, **kwargs):
        """Return fused estimate. Subclasses define the exact signature."""
        raise NotImplementedError


# =============================================================================
# Complementary filter (fixed alpha)
# =============================================================================

class OrientationComplementaryFilter(SensorFusion):
    """
    Fixed-weight complementary filter.

    Blends the AHRS heading (absolute but noisy) with the odometry
    heading (smooth but drifts) using a constant weight::

        fused = odom_theta + alpha * wrap(mag_heading - odom_theta)

    Parameters
    ----------
    alpha : float, 0.0–1.0
        AHRS heading weight.  0 = pure odometry, 1 = pure AHRS.
        Default 0.02 gives gentle long-term drift correction.
    """

    def __init__(self, alpha: float = 0.0) -> None:
        super().__init__()
        self.alpha = max(0.0, min(1.0, float(alpha)))
        self.measurement_type = "orientation"

    def update(
        self,
        odom_theta: float,
        mag_heading: float | None,
        linear_vel: float,
        angular_vel: float,
    ) -> float:
        if mag_heading is None:
            return odom_theta
        return odom_theta + self.alpha * _wrap(mag_heading - odom_theta)


# =============================================================================
# Position complementary filter
# =============================================================================

class PositionComplementaryFilter(SensorFusion):
    """
    Complementary filter for GPS-anchored position fusion.

    Blends GPS (ArUco tag) fixes with wheel odometry using a constant weight::

        fused = odom + alpha * (gps - odom)

    When GPS is stale, dead-reckons from the last valid fix using the odometry
    delta so drift only accumulates since the last GPS update. Falls back to
    raw odometry if no GPS fix has ever been received.

    Parameters
    ----------
    alpha : float, 0.0–1.0
        GPS weight.  0 = pure odometry, 1 = snap directly to GPS each tick.
        Default 0.10.
    """

    def __init__(self, alpha: float = 0.10) -> None:
        super().__init__()
        self.alpha = max(0.0, min(1.0, float(alpha)))
        self._anchor_x_mm      = 0.0
        self._anchor_y_mm      = 0.0
        self._odom_x_at_anchor = 0.0
        self._odom_y_at_anchor = 0.0
        self._anchor_valid     = False
        self.measurement_type = "position"

    def reset(self) -> None:
        """Clear anchor — call when odometry is reset."""
        self._anchor_valid = False

    def update(
        self,
        odom_x: float,
        odom_y: float,
        gps_x: float | None,
        gps_y: float | None,
    ) -> tuple[float, float]:
        if gps_x is not None and gps_y is not None:
            fused_x = odom_x + self.alpha * (gps_x - odom_x)
            fused_y = odom_y + self.alpha * (gps_y - odom_y)
            self._anchor_x_mm      = fused_x
            self._anchor_y_mm      = fused_y
            self._odom_x_at_anchor = odom_x
            self._odom_y_at_anchor = odom_y
            self._anchor_valid     = True
            return fused_x, fused_y
        if self._anchor_valid:
            return (
                self._anchor_x_mm + (odom_x - self._odom_x_at_anchor),
                self._anchor_y_mm + (odom_y - self._odom_y_at_anchor),
            )
        return odom_x, odom_y

