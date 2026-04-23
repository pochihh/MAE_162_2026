"""
path_planner.py — pure-algorithm path planning library
=======================================================
These classes are stateless algorithm helpers. They do NOT own threads or
ROS subscriptions. The Robot class calls compute_velocity() from its own
navigation thread.

To use a planner in your navigation code, just instantiate it and call
compute_velocity() with the current pose and remaining waypoints.
"""

from __future__ import annotations

from collections.abc import Callable
import math
import numpy as np

# =============================================================================
# Base class
# =============================================================================

class PathPlanner:
    """
    Abstract base for path planning algorithms.

    Subclasses implement compute_velocity() and optionally get_obstacles().
    """

    def compute_velocity(
        self,
        pose: tuple[float, float, float],
        waypoints: list[tuple[float, float]],
        max_linear: float,
    ) -> tuple[float, float]:
        """
        Return (linear, angular) velocity command.

          pose       — (x, y, theta_rad) in any consistent unit
          waypoints  — remaining waypoints in the same unit, nearest first
          max_linear — maximum forward speed in that unit/s

        Returns (linear, angular_rad_s).
        """
        raise NotImplementedError

    def get_obstacles(self) -> list:
        """
        Return a list of obstacle positions in the robot's frame.
        Override when a 2D lidar topic is available.
        """
        return []


# =============================================================================
# Pure Pursuit
# =============================================================================

class PurePursuitPlanner(PathPlanner):
    """
    Pure-pursuit path follower for differential drive.

    Steers toward a lookahead point on the path. Works well for smooth
    curves. The lookahead_dist controls the trade-off between responsiveness
    (small) and smoothness (large).

    Parameters:
        lookahead_dist — how far ahead on the path to aim at (same units as pose)
        max_angular    — maximum angular rate (rad/s)
    """

    def __init__(
        self,
        lookahead_dist: float = 150,
        max_angular: float = 2.0,
        goal_tolerance: float = 20.0,
    ) -> None:
        self._lookahead  = lookahead_dist
        self._max_angular = max_angular
        self.goal_tolerance = goal_tolerance

    def compute_velocity(
        self,
        pose: tuple[float, float, float],
        waypoints: list[tuple[float, float]],
        max_linear: float,
    ) -> tuple[float, float]:
        x, y, theta = pose
        tx, ty = self._lookahead_point(x, y, waypoints)
        dx = tx - x
        dy = ty - y

        # Transform the lookahead point into the robot frame.
        x_r = math.cos(theta) * dx + math.sin(theta) * dy
        y_r = -math.sin(theta) * dx + math.cos(theta) * dy
        dist = math.hypot(x_r, y_r)

        if dist < 1e-6:
            return 0.0, 0.0

        # Standard pure-pursuit curvature for a differential-drive robot.
        curvature = 2.0 * y_r / (dist * dist)

        # Slow down for high-curvature turns. The lookahead-scaled term is
        # dimensionless and gives a smooth transition between straight driving
        # and tight cornering.
        forward_scale = max(0.0, x_r / dist)
        curvature_scale = 1.0 + abs(curvature) * self._lookahead
        linear = max_linear * forward_scale / curvature_scale

        if linear <= 1e-6:
            angular = self._max_angular * math.tanh(y_r / max(self._lookahead, 1e-6))
            return 0.0, angular

        angular = curvature * linear
        if abs(angular) > self._max_angular:
            angular = math.copysign(self._max_angular, angular)
            linear = min(linear, abs(angular / curvature)) if abs(curvature) > 1e-6 else linear

        return linear, angular

    def _lookahead_point(
        self, x: float, y: float, waypoints: list[tuple[float, float]]
    ) -> tuple[float, float]:
        """
        Return the first ordered waypoint beyond the lookahead distance.

        The caller is expected to pass the remaining path in route order. That
        avoids Euclidean nearest-point jumps around corners, which otherwise
        make the lookahead target chatter between the incoming and outgoing
        path segments.
        """
        for wx, wy in waypoints:
            if math.hypot(wx - x, wy - y) >= self._lookahead:
                return wx, wy
        return waypoints[-1]
    
    def CurrentTargetReached(self, target_x, target_y, x, y):
        dist_to_target = np.hypot(target_x - x, target_y - y)
        return dist_to_target < self.goal_tolerance


class PurePursuitPlanner2(PathPlanner):
    def __init__(self, lookahead_distance=150.0, max_linear_speed=50.0, goal_tolerance=20.0):
        self.Ld = lookahead_distance
        self.v_max = max_linear_speed  # mm/s
        self.goal_tolerance = goal_tolerance

    def _lookahead_point(self, path, x, y):
        for point in path:
            dx = point[0] - x
            dy = point[1] - y
            if np.hypot(dx, dy) >= self.Ld:
                return point
        return path[-1]
    
    def TargetReached(self, path, x, y):
        goal_x, goal_y = path[-1]
        dist_to_goal = np.hypot(goal_x - x, goal_y - y)
        return dist_to_goal < self.goal_tolerance
        
    def compute_velocity(self, pose, waypoints, max_linear):
        x, y, theta = pose
        goal_x, goal_y = waypoints[-1]
        dist_to_goal = np.hypot(goal_x - x, goal_y - y)
        if dist_to_goal < self.goal_tolerance:
            return 0.0, 0.0  # Stop if within goal tolerance

        target = self._lookahead_point(waypoints, x, y)
        tx, ty = target

        dx = tx - x
        dy = ty - y

        # Transform lookahead point into robot frame
        x_r = np.cos(theta) * dx + np.sin(theta) * dy
        y_r = -np.sin(theta) * dx + np.cos(theta) * dy
        Dist2Target = np.hypot(x_r, y_r)
        if Dist2Target < 1e-6:
            return 0.0, 0.0
        curvature = 2.0 * y_r / (Dist2Target ** 2)

        
        v = min(self.v_max, max_linear) / (1.0 + 2.0 * abs(curvature))  # mm/s
        w = curvature * v  # rad/s
        #print(f"x_r: {x_r:.2f}, y_r: {y_r:.2f}, curvature: {curvature:.4f}, Dist2Target: {Dist2Target:.2f}")
        #print(f"Pose: ({x:.1f}, {y:.1f}, {math.degrees(theta):.1f}°), ", end="")
        #print(f"Target: ({tx:.1f}, {ty:.1f}), Curvature: {curvature:.4f}, v: {v:.2f} mm/s, w: {w:.2f} rad/s")

        return v, w



# =============================================================================
# APF
# =============================================================================

class APFPlanner(PathPlanner):
    """
    Artificial Potential Fields planner.

    Combines an attractive force toward the goal with repulsive forces from
    obstacles. Obstacle data comes from get_obstacles(), which reads the
    lidar topic once it is available.

    This first version is usable today with caller-provided robot-frame
    obstacles. A future lidar/object-detection node can feed live obstacles
    through the Robot obstacle-provider API without changing the planner.
    """

    def __init__(
        self,
        lookahead_dist: float = 200,
        max_linear: float = 200,
        max_angular: float = 2.0,
        repulsion_gain: float = 500.0,
        repulsion_range: float = 300.0,
        goal_tolerance: float = 20.0,
        attraction_gain: float = 1.0,
        heading_gain: float = 2.0,
        obstacle_provider: Callable[[], list[tuple[float, float]]] | None = None,
    ) -> None:
        self._lookahead      = lookahead_dist
        self._max_linear     = max_linear
        self._max_angular    = max_angular
        self._rep_gain       = repulsion_gain
        self._rep_range      = repulsion_range
        self.goal_tolerance  = goal_tolerance
        self._attr_gain      = attraction_gain
        self._heading_gain   = heading_gain
        self._obstacle_provider = obstacle_provider

    def compute_velocity(
        self,
        pose: tuple[float, float, float],
        waypoints: list[tuple[float, float]],
        max_linear: float,
    ) -> tuple[float, float]:
        x, y, theta = pose
        tx, ty = self._lookahead_point(x, y, waypoints)
        dx = tx - x
        dy = ty - y

        # Attractive force points toward the ordered lookahead target in robot frame.
        goal_x_r = math.cos(theta) * dx + math.sin(theta) * dy
        goal_y_r = -math.sin(theta) * dx + math.cos(theta) * dy
        goal_dist = math.hypot(goal_x_r, goal_y_r)
        if goal_dist < 1e-6:
            return 0.0, 0.0

        attr_x = self._attr_gain * goal_x_r / goal_dist
        attr_y = self._attr_gain * goal_y_r / goal_dist

        rep_x = 0.0
        rep_y = 0.0
        nearest_obstacle = self._rep_range
        for obs_x_r, obs_y_r in self.get_obstacles():
            dist = math.hypot(obs_x_r, obs_y_r)
            if dist < 1e-6 or dist >= self._rep_range:
                continue
            nearest_obstacle = min(nearest_obstacle, dist)
            repulse = self._rep_gain * ((1.0 / dist) - (1.0 / self._rep_range)) / (dist * dist)
            rep_x += repulse * (-obs_x_r / dist)
            rep_y += repulse * (-obs_y_r / dist)

        force_x = attr_x + rep_x
        force_y = attr_y + rep_y
        if math.hypot(force_x, force_y) < 1e-6:
            return 0.0, 0.0

        heading_error = math.atan2(force_y, force_x)
        forward_scale = max(0.0, math.cos(heading_error))
        linear_limit = min(float(max_linear), self._max_linear)
        goal_scale = min(1.0, goal_dist / max(self.goal_tolerance * 2.0, 1e-6))
        linear = linear_limit * forward_scale * goal_scale
        if nearest_obstacle < self._rep_range:
            linear *= max(0.0, min(1.0, nearest_obstacle / self._rep_range))
        if force_x <= 0.0:
            linear = 0.0

        angular = self._heading_gain * heading_error
        angular = max(-self._max_angular, min(self._max_angular, angular))
        return linear, angular

    def get_obstacles(self) -> list[tuple[float, float]]:
        if self._obstacle_provider is None:
            return []
        return list(self._obstacle_provider())

    def _lookahead_point(
        self, x: float, y: float, waypoints: list[tuple[float, float]]
    ) -> tuple[float, float]:
        for wx, wy in waypoints:
            if math.hypot(wx - x, wy - y) >= self._lookahead:
                return wx, wy
        return waypoints[-1]


# =============================================================================
# Helper
# =============================================================================

def _wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi
