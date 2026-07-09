"""
path_planner.py — pure-algorithm path planning library
=======================================================
These classes are stateless algorithm helpers. They do NOT own threads or
ROS subscriptions.

``PurePursuitPlanner`` — waypoint-following via lookahead point.
``APFPlanner``         — single-goal Artificial Potential Fields with a
                         live world-frame obstacle cloud (from lidar).
"""

from __future__ import annotations

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
        return self.compute_velocity_to_point(pose, (tx, ty), max_linear)

    def compute_velocity_to_point(
        self,
        pose: tuple[float, float, float],
        target: tuple[float, float],
        max_linear: float,
    ) -> tuple[float, float]:
        x, y, theta = pose
        tx, ty = target
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


# =============================================================================
# APF
# =============================================================================

class APFPlanner:
    """
    Artificial Potential Fields planner — single-goal variant.

    Combines an attractive force toward one goal point with repulsive forces
    from a world-frame obstacle cloud supplied by the caller (e.g. from
    ``LidarScan.to_world_frame()``).  No waypoints, no lookahead buffer, no
    obstacle callbacks — the caller decides what obstacles are relevant.
    """

    def __init__(
        self,
        max_linear: float = 200.0,
        max_angular: float = 2.0,
        repulsion_gain: float = 500.0,
        repulsion_range: float = 300.0,
        goal_tolerance: float = 20.0,
        attraction_gain: float = 1.0,
        heading_gain: float = 2.0,
        force_ema_alpha: float = 0.35,
        robot_front_mm: float = 400.0,
        robot_rear_mm: float = 100.0,
        robot_half_width_mm: float = 200.0,
    ) -> None:
        self._max_linear  = max_linear
        self._max_angular = max_angular
        self._rep_gain    = repulsion_gain
        self._rep_range   = repulsion_range
        self.goal_tolerance = goal_tolerance
        self._attr_gain   = attraction_gain
        self._heading_gain = heading_gain
        self._force_alpha  = float(force_ema_alpha)
        self._front_mm     = float(robot_front_mm)
        self._rear_mm      = float(robot_rear_mm)
        self._half_width   = float(robot_half_width_mm)

        # EMA state for desired heading — None until first call so initial
        # heading is set directly rather than blended from 0.0.
        self._desired_heading: float | None = None
        # Committed tangential side for current avoidance maneuver.
        # Set on first obstacle contact; reset when all obstacles leave range.
        # Prevents choose_left from flipping every tick as the robot rotates.
        self._committed_left: bool | None = None

    def navigate_to_goal(
        self,
        pose: tuple[float, float, float],
        goal: tuple[float, float],
        obstacles: np.ndarray,
    ) -> tuple[float, float]:
        """
        Return ``(linear_mm_s, angular_rad_s)`` toward *goal*, avoiding *obstacles*.

        Parameters
        ----------
        pose      : ``(x_mm, y_mm, theta_rad)`` in world frame
        goal      : ``(x_mm, y_mm)`` target position in world frame
        obstacles : ``(N, 2)`` point array or ``(N, 3)`` disk array
                    in world-frame millimetres. For disks, columns are
                    ``x_mm, y_mm, radius_mm``.
        """
        px, py, theta = pose
        gx, gy = goal

        dx = gx - px
        dy = gy - py
        dist_to_goal = math.hypot(dx, dy)

        if dist_to_goal <= self.goal_tolerance:
            return 0.0, 0.0

        # Attractive force — capped so gain stays bounded far from goal
        attr_scale = self._attr_gain * min(dist_to_goal, self._rep_range)
        attr_x = attr_scale * dx / dist_to_goal
        attr_y = attr_scale * dy / dist_to_goal

        # Repulsive force — vectorised over obstacle cloud.
        #
        # Robot is modelled as a rectangle in its own frame:
        #   forward  [0, front_mm], rearward [0, rear_mm], sides ±half_width.
        # Clearance = distance from rectangle surface to obstacle surface.
        # This is drive-layout agnostic: rear-drive sets front=nose distance,
        # front-drive sets rear=tail distance. Only obstacles ahead of the axle
        # (local_fwd > 0) reduce obstacle_scale so the robot is never stopped
        # by an obstacle it is already passing beside.
        rep_x = 0.0
        rep_y = 0.0
        tan_x = 0.0
        tan_y = 0.0
        obstacle_scale = 1.0
        obs = np.asarray(obstacles)
        if obs.ndim != 2 or obs.shape[0] == 0:
            self._committed_left = None
        if obs.ndim == 2 and obs.shape[0] > 0:
            centers = obs[:, :2]
            radii = obs[:, 2] if obs.shape[1] >= 3 else np.zeros(obs.shape[0], dtype=float)

            cos_th = math.cos(theta)
            sin_th = math.sin(theta)

            # Obstacle centres in robot-local frame
            to_obs_x = centers[:, 0] - px
            to_obs_y = centers[:, 1] - py
            local_fwd   = to_obs_x * cos_th + to_obs_y * sin_th
            local_right = to_obs_x * sin_th - to_obs_y * cos_th

            # Nearest point on the robot rectangle to each obstacle (local frame)
            clamped_fwd   = np.clip(local_fwd,   -self._rear_mm,   self._front_mm)
            clamped_right = np.clip(local_right, -self._half_width, self._half_width)

            # Convert nearest rectangle point back to world frame for force direction
            nearest_x = px + clamped_right * sin_th + clamped_fwd * cos_th
            nearest_y = py - clamped_right * cos_th + clamped_fwd * sin_th

            # Vector from each obstacle toward nearest rectangle surface point
            fx_raw = nearest_x - centers[:, 0]
            fy_raw = nearest_y - centers[:, 1]
            rect_dists = np.maximum(np.sqrt(fx_raw * fx_raw + fy_raw * fy_raw), 1e-6)

            # Physical clearance between rectangle surface and obstacle surface
            boundary_dists = np.maximum(rect_dists - radii, 0.0)
            in_range = boundary_dists < self._rep_range

            if np.any(in_range):
                d = np.maximum(boundary_dists[in_range], 1e-6)
                proximity = 1.0 - (d / self._rep_range)
                mag = self._rep_gain * proximity * proximity

                ux = fx_raw[in_range] / rect_dists[in_range]
                uy = fy_raw[in_range] / rect_dists[in_range]
                rep_x = float(np.sum(mag * ux))
                rep_y = float(np.sum(mag * uy))

                # Tangential escape — commit to a side on first contact and
                # hold it for the whole maneuver. Recomputing choose_left every
                # tick causes it to flip as the robot rotates, producing wobble.
                left_tx = -uy
                left_ty =  ux
                right_tx =  uy
                right_ty = -ux
                if self._committed_left is None:
                    goal_ux = dx / dist_to_goal
                    goal_uy = dy / dist_to_goal
                    left_score  = float(np.sum(left_tx  * goal_ux + left_ty  * goal_uy))
                    right_score = float(np.sum(right_tx * goal_ux + right_ty * goal_uy))
                    self._committed_left = left_score >= right_score

                tangent_mag = 0.25 * self._rep_gain * proximity * proximity
                if self._committed_left:
                    tan_x = float(np.sum(tangent_mag * left_tx))
                    tan_y = float(np.sum(tangent_mag * left_ty))
                else:
                    tan_x = float(np.sum(tangent_mag * right_tx))
                    tan_y = float(np.sum(tangent_mag * right_ty))

                # Speed scaling: only forward obstacles (ahead of axle) reduce
                # linear velocity. Side/rear obstacles steer but don't stop.
                # Floor of 0.15 prevents a full stop when touching the front
                # face — the robot keeps creeping while rotating clear.
                fwd_mask = local_fwd[in_range] > 0
                if np.any(fwd_mask):
                    nearest_fwd_boundary = float(np.min(boundary_dists[in_range][fwd_mask]))
                    obstacle_scale = max(0.15, min(1.0, nearest_fwd_boundary / self._rep_range))
                # else: no forward obstacle → obstacle_scale stays 1.0
            else:
                self._committed_left = None  # obstacle cleared — fresh choice next time

        # Radial repulsion is intentionally excluded from the force direction.
        # The robot is already slowed by obstacle_scale; adding a backward
        # component to the heading target is what causes over-turning.
        # The tangential component alone steers the robot around the obstacle.
        force_x = attr_x + tan_x
        force_y = attr_y + tan_y

        if math.hypot(force_x, force_y) < 1e-6:
            return 0.0, 0.0

        # EMA on the desired heading angle (not force components) so opposite
        # avoidance and attractive vectors can't cancel inside the filter.
        desired_raw = math.atan2(force_y, force_x)
        if self._desired_heading is None:
            self._desired_heading = desired_raw
        else:
            delta = (desired_raw - self._desired_heading + math.pi) % (2.0 * math.pi) - math.pi
            self._desired_heading = (self._desired_heading + self._force_alpha * delta + math.pi) % (2.0 * math.pi) - math.pi
        desired = self._desired_heading
        ang_err  = (desired - theta + math.pi) % (2.0 * math.pi) - math.pi

        forward_scale = max(0.0, math.cos(ang_err))
        goal_scale    = min(1.0, dist_to_goal / max(2.0 * self.goal_tolerance, 1.0))
        linear  = self._max_linear * forward_scale * goal_scale * obstacle_scale

        angular = self._heading_gain * ang_err
        angular = max(-self._max_angular, min(self._max_angular, angular))

        return linear, angular


class LeashedAPFPlanner:
    """
    Local goal-seeking planner using a leashed virtual target.

    A virtual target point evolves under a point-APF field in world frame.
    The real robot then tracks that target with a pure-pursuit-derived
    single-point tracker. The virtual target is constrained to remain within
    a leash cone in front of the robot.
    """

    def __init__(
        self,
        *,
        max_linear: float = 200.0,
        max_angular: float = 1.0,
        target_speed: float = 200.0,
        repulsion_gain: float = 800.0,
        repulsion_range: float = 700.0,
        goal_tolerance: float = 20.0,
        attraction_gain: float = 1.0,
        force_ema_alpha: float = 0.35,
        leash_length_mm: float = 400.0,
        leash_half_angle_deg: float = 60.0,
        inflation_margin_mm: float = 200.0,
        tracker_lookahead_mm: float = 150.0,
    ) -> None:
        self._max_linear = float(max_linear)
        self._max_angular = float(max_angular)
        self._target_speed = float(target_speed)
        self._rep_gain = float(repulsion_gain)
        self._rep_range = float(repulsion_range)
        self.goal_tolerance = float(goal_tolerance)
        self._attr_gain = float(attraction_gain)
        self._force_alpha = float(force_ema_alpha)
        self._leash_length = float(leash_length_mm)
        self._leash_half_angle = math.radians(float(leash_half_angle_deg))
        self._inflation_margin = float(inflation_margin_mm)
        self._tracker = PurePursuitPlanner(
            lookahead_dist=float(tracker_lookahead_mm),
            max_angular=float(max_angular),
            goal_tolerance=float(goal_tolerance),
        )

        self._virtual_target: tuple[float, float] | None = None
        self._force_ema: np.ndarray | None = None

    def reset(self) -> None:
        self._virtual_target = None
        self._force_ema = None

    def get_virtual_target(self) -> tuple[float, float] | None:
        return self._virtual_target

    def navigate_to_goal(
        self,
        pose: tuple[float, float, float],
        goal: tuple[float, float],
        obstacles: np.ndarray,
        dt: float,
    ) -> tuple[float, float]:
        px, py, _theta = pose
        gx, gy = goal
        if math.hypot(gx - px, gy - py) <= self.goal_tolerance:
            self._virtual_target = goal
            return 0.0, 0.0

        target = self.update_virtual_target(pose, goal, obstacles, dt)
        return self._tracker.compute_velocity_to_point(pose, target, self._max_linear)

    def update_virtual_target(
        self,
        pose: tuple[float, float, float],
        goal: tuple[float, float],
        obstacles: np.ndarray,
        dt: float,
    ) -> tuple[float, float]:
        px, py, theta = pose
        gx, gy = goal
        dt = max(1e-3, float(dt))

        if self._virtual_target is None:
            self._virtual_target = (float(px), float(py))

        vx, vy = self._virtual_target
        raw_force = self._compute_force((vx, vy), goal, obstacles)

        if self._force_ema is None:
            filtered_force = raw_force
        else:
            filtered_force = (1.0 - self._force_alpha) * self._force_ema + self._force_alpha * raw_force
        self._force_ema = filtered_force

        force_norm = float(np.linalg.norm(filtered_force))
        goal_dist = math.hypot(gx - vx, gy - vy)
        if goal_dist <= self.goal_tolerance:
            candidate = (gx, gy)
        elif force_norm <= 1e-6:
            candidate = (vx, vy)
        else:
            direction = filtered_force / force_norm
            step_speed = min(self._target_speed, goal_dist / dt)
            candidate = (
                vx + float(direction[0]) * step_speed * dt,
                vy + float(direction[1]) * step_speed * dt,
            )

        constrained = self._apply_leash(candidate, pose)
        self._virtual_target = constrained
        return constrained

    def _compute_force(
        self,
        point: tuple[float, float],
        goal: tuple[float, float],
        obstacles: np.ndarray,
    ) -> np.ndarray:
        vx, vy = point
        gx, gy = goal

        goal_vec = np.array([gx - vx, gy - vy], dtype=float)
        goal_dist = float(np.linalg.norm(goal_vec))
        if goal_dist > 1e-6:
            attr = self._attr_gain * (goal_vec / goal_dist)
        else:
            attr = np.zeros(2, dtype=float)

        rep = np.zeros(2, dtype=float)
        nearest_obs: tuple[float, float, float] | None = None
        nearest_boundary = float("inf")
        obs = np.asarray(obstacles, dtype=float)
        if obs.ndim == 2 and obs.shape[0] > 0:
            for row in obs:
                ox = float(row[0])
                oy = float(row[1])
                radius = float(row[2]) if row.shape[0] >= 3 else 0.0
                eff_radius = radius + self._inflation_margin
                away = np.array([vx - ox, vy - oy], dtype=float)
                center_dist = float(np.linalg.norm(away))
                if center_dist <= 1e-6:
                    away = np.array([0.0, 1.0], dtype=float)
                    center_dist = 1e-6
                boundary = center_dist - eff_radius
                if boundary >= self._rep_range:
                    continue
                if boundary < nearest_boundary:
                    nearest_boundary = boundary
                    nearest_obs = (ox, oy, eff_radius)
                direction = away / center_dist
                clearance = max(boundary, 1.0)
                rep_mag = self._rep_gain * max(0.0, (1.0 / clearance) - (1.0 / self._rep_range))
                rep += direction * rep_mag

        total = attr + rep

        # APF symmetry can leave a centered obstacle with no lateral escape.
        # Add a small deterministic tangent only when the total force is nearly
        # singular near a blocking obstacle.
        if nearest_obs is not None and np.linalg.norm(total) < 0.25:
            ox, oy, _eff_radius = nearest_obs
            away = np.array([vx - ox, vy - oy], dtype=float)
            norm = float(np.linalg.norm(away))
            if norm > 1e-6:
                away /= norm
                tangent = np.array([-away[1], away[0]], dtype=float)
                total += 0.15 * self._attr_gain * tangent

        return total

    def _apply_leash(
        self,
        target: tuple[float, float],
        pose: tuple[float, float, float],
    ) -> tuple[float, float]:
        px, py, theta = pose
        tx, ty = target
        dx = tx - px
        dy = ty - py

        local_x = math.cos(theta) * dx + math.sin(theta) * dy
        local_y = -math.sin(theta) * dx + math.cos(theta) * dy
        distance = math.hypot(local_x, local_y)
        angle = math.atan2(local_y, local_x)

        clipped_distance = min(distance, self._leash_length)
        clipped_angle = max(-self._leash_half_angle, min(self._leash_half_angle, angle))

        local_x = clipped_distance * math.cos(clipped_angle)
        local_y = clipped_distance * math.sin(clipped_angle)

        world_x = px + math.cos(theta) * local_x - math.sin(theta) * local_y
        world_y = py + math.sin(theta) * local_x + math.cos(theta) * local_y
        return float(world_x), float(world_y)


class PurePursuitPlannerWithAvoidance(PathPlanner):
    def __init__(self,
            lookahead_distance: float=100.0,
            max_linear_speed: float=130.0,
            max_angular_speed: float=1.0,
            goal_tolerance: float=20.0,
            obstacles_range: float=400.0,
            view_angle: float=np.pi/2,
            safe_dist: float=150.0,
            avoidance_delay: int=200,
            offset: float=120.0,
            x_L: float=0.0,
            lane_width: float=500.0,
            alpha_Ld: float=0.8,
            obstacle_avoidance: bool = True,
            ):
        self.Ld = lookahead_distance
        self.raw_LD = lookahead_distance
        self.v_max = max_linear_speed  # mm/s
        self.w_max = max_angular_speed  # rad/s
        self.goal_tolerance = goal_tolerance
        self.obstacles_range = obstacles_range
        self.view_angle = view_angle
        self.safe_dist = safe_dist
        self.alpha_Ld = alpha_Ld
        self.obstacle_avoidance = obstacle_avoidance
        self.x_L = x_L
        self.lane_width = lane_width
        self.offset = offset

        self.avoidance_active = False
        self.avoidance_counter = 0
        self.avoidance_delay = avoidance_delay

        # self.current_lane = 'Center'
        self.current_lane = 'Left'

    def set_path(self, path: list[tuple[float, float]]):
        self.raw_path = path.copy()
        if self.current_lane == 'Center':
            self.remaining_path = path.copy()
        elif self.current_lane == 'Left':
            self.remaining_path = []
            for i in range(len(self.raw_path)):
                x_, y_ = self.raw_path[i]
                self.remaining_path.append((x_-self.offset, y_))
        elif self.current_lane == 'Right':
            self.remaining_path = []
            for i in range(len(self.raw_path)):
                x_, y_ = self.raw_path[i]
                self.remaining_path.append((x_+self.offset, y_))

    def _advance_remaining_path(self,
        x: float,
        y: float,
    ) -> list[tuple[float, float]]:
        
        while len(self.remaining_path) > 1:
            next_x_mm, next_y_mm = self.remaining_path[0]
            if np.hypot(x-next_x_mm, y-next_y_mm) > self.Ld:
                break
            self.remaining_path.pop(0)
            self.raw_path.pop(0)

            if self.avoidance_active:
                self.avoidance_active = False
                self.Ld = self.raw_LD
                self.avoidance_counter = 0

        return self.remaining_path

    def _lookahead_point(self, path, x, y):
        path = np.array(path)
        position = np.array([x, y])

        for i in range(len(path)):
            dist = np.linalg.norm(path[i,:] - position)
            if dist >= self.Ld:
                return path[i]

        return path[-1]
    
    def TargetReached(self, path, x, y):
        if self.avoidance_active:
            return False # in avoidance mode, we don't check goal reached condition to prevent the robot from stopping before reaching the goal due to the added waypoints for obstacle avoidance, which may cause the robot to think it's close enough to the goal when it's actually still far away.
        goal_x, goal_y = path[0]
        dist_to_goal = np.hypot(goal_x - x, goal_y - y)
        return (dist_to_goal < self.goal_tolerance)

    def gen_obstacle_waypoint(self, pose, obstacles_r):
        # Step 1: Obtain current state: Obtain pose and obstacles in robot frame based on your lidar and robot configurations.
        x, y, theta = pose
        if len(obstacles_r) > 0:
            # lidar orientation due to installation is 180 deg rotated from robot forward, so rotate obstacles accordingly.
            obstacles_r = (np.array([[np.cos(np.pi), -np.sin(np.pi)], [np.sin(np.pi), np.cos(np.pi)]]) @ obstacles_r.T).T
            obstacles_r[:,1] *= -1
            
            # since some robot parts (e.g., the arm) may cause obstacles to be detected, we can filter out those obstacles behind the lidar.
            obstacles_r = obstacles_r[np.abs(np.arctan2(obstacles_r[:,1],obstacles_r[:,0])) <= self.view_angle,:] # only consider obstacles in front of the robot within 180 deg FOV, which can help prevent the robot from being too conservative by reacting to obstacles behind it that are not in its path.

            # consider the lidar offset from the robot center
            # lidar_offset_mm = 100.0
            # obstacles_r = obstacles_r + np.array([[lidar_offset_mm, 0],])

            # Filter out obstacles outside of detecting range.
            dists = np.linalg.norm(obstacles_r, axis=1)
            obstacles_r = obstacles_r[(dists < self.obstacles_range)]

            # Step 2: Obstacle filtering and path modification
            # Transform obstacles from robot frame to world frame
            obstacles = (np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]) @ obstacles_r.T).T + np.array([[x, y],])

            # Obstacle filtering by lane width.
            obstacles_r = obstacles_r[np.abs(obstacles[:,0]-self.x_L)<self.lane_width,:]
            obstacles = obstacles[np.abs(obstacles[:,0]-self.x_L)<self.lane_width,:]

            # Modify path waypoints that are too close to the obstacles to prevent the robot from trying to track those waypoints and colliding with the obstacles.
            if np.any(np.sqrt(np.sum((np.float64([self.remaining_path[0]])-obstacles)**2, 1)) < self.safe_dist):
                self.remaining_path[0] = ((self.remaining_path[0][0]+self.remaining_path[1][0])/2, (self.remaining_path[0][1]+self.remaining_path[1][1])/2)
                self.raw_path[0] = ((self.raw_path[0][0]+self.raw_path[1][0])/2, (self.raw_path[0][1]+self.raw_path[1][1])/2)

            if (len(obstacles_r) > 0)  and (self.avoidance_counter <= 0):
                # Step 3: Find the cloest obstacle, and decide which lane to switch.
                dists = np.linalg.norm(obstacles_r, axis=1)
                # min_dist = np.min(dists)
                arg_dist = np.argmin(dists)
                closest_pt = obstacles[arg_dist,:] # closest obstacle point in world frame

                change_lane = False
                if (closest_pt[0] < self.x_L and self.current_lane!='Right') or (closest_pt[0] > self.x_L and self.current_lane!='Left'):
                    change_lane = True
                    # reduce lookahead distance to track added waypoints more precisely.
                    self.Ld = self.raw_LD * self.alpha_Ld

                    # keep avoidance active for a few cycles to ensure the robot reacts to the obstacle.
                    self.avoidance_counter = self.avoidance_delay
                    self.avoidance_active = True

                # Generate new waypoints based on the desired waypoints on the center lane.
                if change_lane:
                    self.remaining_path = []
                    
                    # ALGORITHM UPDATE: Create a smooth diagonal merge
                    # By making the forward distance slightly larger than the lateral offset, 
                    # we keep the transition angle under 45°, allowing forward speed to continue.
                    merge_forward_dist = self.offset * 1.2 
                    merge_y = y + merge_forward_dist
                    if closest_pt[0] < self.x_L:
                        self.current_lane = 'Right'
                        target_x = x + self.offset
                    else:
                        self.current_lane = 'Left'
                        target_x = x - self.offset
                    # Inject a midpoint to give Pure Pursuit a diagonal breadcrumb trail to follow
                    mid_x = x + ((target_x - x) / 2.0)
                    mid_y = y + (merge_forward_dist / 2.0)
                    self.remaining_path.append((mid_x, mid_y))
                    self.remaining_path.append((target_x, merge_y)) 
                    # Append the rest of the shifted path
                    for i in range(len(self.raw_path)):
                        x_, y_ = self.raw_path[i]
                        if y_ > merge_y: 
                            if self.current_lane == 'Right':
                                self.remaining_path.append((x_ + self.offset, y_))
                            else:
                                self.remaining_path.append((x_ - self.offset, y_))
                    print('Change Lane!!! Current lane is:', self.current_lane)
                    # Reduce lookahead distance to track the diagonal precisely.
                    self.Ld = self.raw_LD * self.alpha_Ld
                    # Keep avoidance active for a few cycles.
                    self.avoidance_counter = self.avoidance_delay
                    self.avoidance_active = True

        if self.avoidance_counter > 0:
            self.avoidance_counter -= 1

    def compute_velocity(self, pose, obstacles_r: np.nparray):
        # Note that the input obstacle point cloud is in robot frame
        x, y, theta = pose
        self._advance_remaining_path(x,y)
        
        if self.TargetReached(self.remaining_path,x,y):
            return 0.0, 0.0  # Stop if within goal tolerance

        if self.obstacle_avoidance:
            self.gen_obstacle_waypoint(pose, obstacles_r)

        target = self._lookahead_point(self.remaining_path, x, y)
        tx, ty = target

        dx = tx - x
        dy = ty - y

        # Transform to robot frame
        x_r = np.cos(theta) * dx + np.sin(theta) * dy
        y_r = -np.sin(theta) * dx + np.cos(theta) * dy
        dist = math.hypot(x_r, y_r)

        if dist < 1e-6:
            return 0.0, 0.0

        # Standard pure-pursuit curvature for a differential-drive robot.
        curvature = 2.0 * y_r / (dist * dist)

        # Slow down for high-curvature turns. The lookahead-scaled term is
        # dimensionless and gives a smooth transition between straight driving
        # and tight cornering.
        forward_scale = max(0.0, x_r / dist)
        curvature_scale = 1.0 + abs(curvature) * self.Ld
        linear = self.v_max * forward_scale / curvature_scale

        if linear <= 1e-6:
            angular = self.w_max * math.tanh(y_r / max(self.Ld, 1e-6))
            return 0.0, angular

        angular = curvature * linear
        if abs(angular) > self.w_max:
            angular = math.copysign(self.w_max, angular)
            linear = min(linear, abs(angular / curvature)) if abs(curvature) > 1e-6 else linear

        return linear, angular

    def motion(self, pose, v, w, dt):
        x, y, theta = pose
        pose[0] += v * np.cos(theta) * dt
        pose[1] += v * np.sin(theta) * dt
        pose[2] += w * dt
        return pose


# =============================================================================
# Helper
# =============================================================================

def _wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi



# =============================================================================
# Dynamic Window Approach (DWA)
# =============================================================================

class DWAPlanner(PathPlanner):
    """
    Dynamic Window Approach path planner for local obstacle avoidance.
    
    Generates a fan of feasible velocity trajectories based on the robot's
    kinematic limits, scores them against a cost function (heading, clearance, 
    velocity), and selects the optimal (v, w) command.
    """
    def __init__(
        self,
        lookahead_dist: float,
        max_linear_speed: float,
        max_angular_speed: float,
        max_linear_acc: float,
        max_angular_acc: float,
        goal_tolerance: float,
        gains_of_costs: list[float],
        dt: float,
        predict_time: float,
        predict_velocity_samples_resolution: list[int],
        robot_front_mm: float,       # <-- Replaced robot_radius
        robot_rear_mm: float,        # <-- 
        robot_half_width_mm: float,  # <-- 
        obstacles_range: float,
        ttc_weight: float = 0.0,
    ):
        self.lookahead_dist = lookahead_dist
        self.max_v = max_linear_speed
        self.max_w = max_angular_speed
        self.max_acc_v = max_linear_acc
        self.max_acc_w = max_angular_acc
        self.goal_tolerance = goal_tolerance
        self.gains = gains_of_costs
        self.dt = dt
        self.predict_time = predict_time
        self.v_res = max(1, predict_velocity_samples_resolution[0])
        self.w_res = max(1, predict_velocity_samples_resolution[1])
        
        # Rectangular footprint bounds relative to the rear-axle origin
        self.robot_front_mm = robot_front_mm
        self.robot_rear_mm = robot_rear_mm
        self.robot_half_width_mm = robot_half_width_mm
        
        self.obstacles_range = obstacles_range
        self.ttc_weight = ttc_weight

        # Lidar Hardware specifications - match hardware_map.py
        self.lidar_mount_x = 336.55
        self.lidar_mount_y = 0.0
        self.lidar_range_min = 75.0
        self.lidar_range_max = 1500.0
        self.lidar_fov_min_rad = math.radians(-90.0)
        self.lidar_fov_max_rad = math.radians(90.0)

    def TargetReached(self, path: list[tuple[float, float]], current_x: float, current_y: float) -> bool:
        if not path:
            return True
        final_x, final_y = path[-1]
        return math.hypot(final_x - current_x, final_y - current_y) <= self.goal_tolerance

    def _get_local_goal(self, path: list[tuple[float, float]], pose: tuple[float, float, float]) -> tuple[float, float]:
        rx, ry, _ = pose
        
        # Auto-scale the aiming distance:
        # Ensures DWA always aims at a point further than its longest trajectory,
        # preventing the "overshoot panic" even if lookahead_dist is very small.
        predict_dist = self.max_v * self.predict_time
        min_aim_dist = max(self.lookahead_dist, predict_dist)
        
        best_goal = path[-1]
        for pt in path:
            if math.hypot(pt[0] - rx, pt[1] - ry) >= min_aim_dist:
                best_goal = pt
                break
        return best_goal

    def _filter_and_transform_obstacles(self, raw_obstacles_r: np.ndarray, pose: tuple[float, float, float]) -> np.ndarray:
        """Filters lidar points by FOV/Range and transforms to Global frame."""
        if raw_obstacles_r.size == 0:
            return np.array([])

        x_lidar = raw_obstacles_r[:, 0]
        y_lidar = raw_obstacles_r[:, 1]

        # 1. Filter based on FOV and Range
        ranges = np.hypot(x_lidar, y_lidar)
        angles = np.arctan2(y_lidar, x_lidar)
        
        valid_mask = (
            (ranges >= self.lidar_range_min) &
            (ranges <= self.lidar_range_max) &
            (angles >= self.lidar_fov_min_rad) &
            (angles <= self.lidar_fov_max_rad)
        )

        # # --- RAMP PERCEPTION MASK: not used anymore --- 
        # if getattr(self, 'ramp_mask_active', False):
        #     # Carve out a blind spot (+/- x degrees) dead ahead to ignore the ramp incline
        #     blind_spot_rad = math.radians(60.0)
        #     valid_mask &= (np.abs(angles) > blind_spot_rad)
        # # -----------------------------------
        
        x_filtered = x_lidar[valid_mask]
        y_filtered = y_lidar[valid_mask]

        if len(x_filtered) == 0:
            return np.array([])

        # 2. Shift to Robot Center (apply mount offset)
        x_robot = x_filtered + self.lidar_mount_x
        y_robot = y_filtered + self.lidar_mount_y

        # 3. Transform to Global Frame
        gx, gy, gtheta = pose
        cos_theta = math.cos(gtheta)
        sin_theta = math.sin(gtheta)

        global_x = gx + (x_robot * cos_theta - y_robot * sin_theta)
        global_y = gy + (x_robot * sin_theta + y_robot * cos_theta)

        return np.column_stack((global_x, global_y))

    def _calc_dynamic_window(self, vel: tuple[float, float, float]) -> tuple[float, float, float, float]:
        v_current = vel[0]  
        w_current = vel[2]  

        # Expand the acceleration window. If we strictly use the FSM dt (0.02s), 
        # the planner restricts the rover to tiny adjustments and forbids hard swerves.
        window_time = 0.5 

        # Specification limits [v_min, v_max, w_min, w_max]
        Vs = [0.0, self.max_v, -self.max_w, self.max_w]

        # Dynamic limits based on acceleration
        Vd = [
            v_current - self.max_acc_v * window_time,
            v_current + self.max_acc_v * window_time,
            w_current - self.max_acc_w * window_time,
            w_current + self.max_acc_w * window_time
        ]

        return (
            max(Vs[0], Vd[0]),
            min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]),
            min(Vs[3], Vd[3])
        )

    def _predict_trajectory(self, pose: tuple[float, float, float], v: float, w: float) -> np.ndarray:
        x, y, theta = pose
        time = 0.0
        trajectory = []
        
        while time <= self.predict_time:
            x += v * math.cos(theta) * self.dt
            y += v * math.sin(theta) * self.dt
            theta += w * self.dt
            trajectory.append([x, y, theta])
            time += self.dt
            
        return np.array(trajectory)

    def compute_velocity(self, path: list[tuple[float, float]], pose: tuple[float, float, float], vel: tuple[float, float, float], raw_obstacles: np.ndarray, period: float) -> tuple[float, float]:
        self.dt = period 
        goal = self._get_local_goal(path, pose)
        global_obstacles = self._filter_and_transform_obstacles(raw_obstacles, pose)
        dw = self._calc_dynamic_window(vel)

        best_v = 0.0
        best_w = 0.0
        min_cost = float('inf')

        v_samples = np.linspace(dw[0], dw[1], self.v_res)
        w_samples = np.linspace(dw[2], dw[3], self.w_res)

        for v in v_samples:
            for w in w_samples:
                
                # ---------------------------------------------------------
                # ANTI-DONUT RULE
                # If a trajectory turns more than 180 degrees in the prediction 
                # window, discard it. This prevents the robot from driving in 
                # circles to "cheat" the heading cost.
                # ---------------------------------------------------------
                if abs(w * self.predict_time) > math.pi:
                    continue
                
                trajectory = self._predict_trajectory(pose, v, w)
                final_pose = trajectory[-1]


                # 1. Heading Cost (Normalized 0.0 to 1.0)
                # 0.0 = pointing exactly at goal, 1.0 = pointing completely opposite
                angle_to_goal = math.atan2(goal[1] - final_pose[1], goal[0] - final_pose[0])
                heading_diff = abs(_wrap_angle(angle_to_goal - final_pose[2]))
                heading_cost = self.gains[0] * (heading_diff / math.pi)
                
                # 2. Clearance Cost (Normalized 0.0 to 1.0)
                clearance_cost = 0.0
                if global_obstacles.size > 0:
                    min_dist = float('inf')
                    
                    for i in range(len(trajectory)):
                        tx, ty, ttheta = trajectory[i]
                        cos_th = math.cos(ttheta)
                        sin_th = math.sin(ttheta)

                        dx = global_obstacles[:, 0] - tx
                        dy = global_obstacles[:, 1] - ty
                        local_x = dx * cos_th + dy * sin_th
                        local_y = -dx * sin_th + dy * cos_th

                        clamped_x = np.clip(local_x, -self.robot_rear_mm, self.robot_front_mm)
                        clamped_y = np.clip(local_y, -self.robot_half_width_mm, self.robot_half_width_mm)

                        distances = np.hypot(local_x - clamped_x, local_y - clamped_y)
                        step_min_dist = np.min(distances)

                        if step_min_dist < min_dist:
                            min_dist = step_min_dist

                    if min_dist <= 0.0:
                        continue  # Skip this trajectory; it causes a collision
                    elif min_dist < self.obstacles_range:
                        # 1.0 = practically touching, 0.0 = obstacle is at the outer edge of obstacles_range
                        clearance_cost = self.gains[1] * ((self.obstacles_range - min_dist) / self.obstacles_range)
                
                # 3. Velocity Cost (Normalized 0.0 to 1.0)
                # 0.0 = driving at max speed, 1.0 = fully stopped
                vel_cost = self.gains[2] * ((self.max_v - v) / self.max_v)

                total_cost = heading_cost + clearance_cost + vel_cost

                if total_cost < min_cost:
                    min_cost = total_cost
                    best_v = v
                    best_w = w

        # Emergency fallback: Stop and gently spin to find a clear path
        # if min_cost == float('inf'):
        #     return 0.0, self.max_w * 0.5
        
        # Emergency fallback: Employ collision handling if trapped or if best path is to stay still
        if min_cost == float('inf') or (best_v == 0.0 and best_w == 0.0):
            print("Collision Detected - Repositioning...")
            return None, None

        return best_v, best_w