#!/usr/bin/env python3
import time
import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def angle_wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quat(q) -> float:
    # geometry_msgs/Quaternion -> yaw (rad)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PathFollowerExplore(Node):
    """
    Exploration path follower (ROS2 Jazzy):
      - Sub: /planned_path (nav_msgs/Path), /odom (nav_msgs/Odometry), /scan (LaserScan), /frontier_goal (PoseStamped)
      - Pub: /cmd_vel (Twist), /replan (Bool pulse), /frontier_next (Bool pulse)

    Internal control uses normalized commands in [-1..1], then publish_cmd() scales to TurtleBot limits:
      linear.x in [0..max_linear_mps]
      angular.z in [-max_angular_radps..+max_angular_radps]    , taken from my previous robot
    """

    FOLLOW_PATH = 0
    AVOID_TURN = 1
    AVOID_DRIVE = 2

    def __init__(self):
        super().__init__("path_follower_explore")

        # ---------------- Topics ----------------
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("frontier_goal_topic", "/frontier_goal")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("replan_topic", "/replan")
        self.declare_parameter("frontier_next_topic", "/frontier_next")

        # ---------------- Control loop ----------------
        self.declare_parameter("control_rate_hz", 20.0)

        # ---------------- Tolerances ----------------
        self.declare_parameter("waypoint_tolerance_m", 0.25)
        self.declare_parameter("goal_tolerance_m", 0.05)

        # ---------------- TurtleBot cmd_vel limits ----------------
        # Waffle/Waffle Pi often uses max linear ~0.26 and max angular ~1.82
        self.declare_parameter("max_linear_mps", 0.19)  # max 0.26
        self.declare_parameter("max_angular_radps", 1.6) # 1.82

        # ---------------- Differential drive scaling (kept ) ----------------
        self.declare_parameter("track_width_m", 0.287)          # distance betwee wheels
        self.declare_parameter("max_wheel_speed_mps", 0.75)     # internal scaling only
        self.declare_parameter("K_v", 3.5)
        self.declare_parameter("K_w", 2.2)
        self.declare_parameter("turn_in_place_threshold_rad", 1.0)
        self.declare_parameter("invert_turn", True)

        # Goal deadline (timeout per frontier goal)
        self.declare_parameter("frontier_goal_timeout_s", 35.0) # was 35
        self.declare_parameter("frontier_next_timeout_s", 50.0)
        self.frontier_goal_timeout_s = float(self.get_parameter("frontier_goal_timeout_s").value)

        self.goal_received_t: Optional[float] = None
        self.goal_timed_out = False

        # Rate limiting on normalized commands
        self.declare_parameter("max_forward_rate", 2.0)
        self.declare_parameter("max_turn_rate", 4.0)

        # ---------------- Reactive avoidance ----------------
        self.declare_parameter("avoid_turn_sign", -1.0) #  -1 for real robot
        self.declare_parameter("safety_distance_m", 0.35) # was 0.35,sim 30
        self.declare_parameter("front_cone_deg", 60.0)
        self.declare_parameter("side_cone_deg", 80.0)
        self.declare_parameter("avoid_turn_cmd", 0.20)
        self.declare_parameter("avoid_drive_fwd_cmd", 0.75)
        self.declare_parameter("avoid_drive_turn_cmd", 0.25)
        self.declare_parameter("clear_hysteresis_m", 0.45)
        self.declare_parameter("scan_stale_s", 0.5)
        self.declare_parameter("max_rejoin_waypoints", 70)
        self.declare_parameter("avoid_timeout_s", 8.0)

        self.declare_parameter("avoid_min_turn_time_s", 1.0)
        self.declare_parameter("avoid_cooldown_s", 1.9)
        self.declare_parameter("avoid_commit_fwd_s", 2.0)
        self.declare_parameter("avoid_commit_fwd_cmd", 0.15)

        self.declare_parameter("bearing_free_margin_m", 0.7)
        self.declare_parameter("bearing_max_abs_deg", 70.0)

        # ---------------- Stuck / replan / next ----------------
        self.declare_parameter("stall_timeout_s", 15.0)
        self.declare_parameter("progress_eps_m", 0.10)
        self.declare_parameter("replan_cooldown_s", 2.0)
        self.declare_parameter("max_replans_before_next", 5)


        # ---------------- Read params ----------------
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.frontier_goal_topic = str(self.get_parameter("frontier_goal_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.replan_topic = str(self.get_parameter("replan_topic").value)
        self.frontier_next_topic = str(self.get_parameter("frontier_next_topic").value)

        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)

        self.waypoint_tolerance_m = float(self.get_parameter("waypoint_tolerance_m").value)
        self.goal_tolerance_m = float(self.get_parameter("goal_tolerance_m").value)

        self.max_linear_mps = float(self.get_parameter("max_linear_mps").value)
        self.max_angular_radps = float(self.get_parameter("max_angular_radps").value)

        self.track_width_m = float(self.get_parameter("track_width_m").value)
        self.max_wheel_speed_mps = float(self.get_parameter("max_wheel_speed_mps").value)
        self.K_v = float(self.get_parameter("K_v").value)
        self.K_w = float(self.get_parameter("K_w").value)
        self.turn_in_place_threshold = float(self.get_parameter("turn_in_place_threshold_rad").value)
        self.invert_turn = bool(self.get_parameter("invert_turn").value)

        self.max_forward_rate = float(self.get_parameter("max_forward_rate").value)
        self.max_turn_rate = float(self.get_parameter("max_turn_rate").value)

        self.avoid_turn_sign = float(self.get_parameter("avoid_turn_sign").value)
        self.safety_distance_m = float(self.get_parameter("safety_distance_m").value)
        self.front_cone_rad = math.radians(float(self.get_parameter("front_cone_deg").value))
        self.side_cone_rad = math.radians(float(self.get_parameter("side_cone_deg").value))
        self.avoid_turn_cmd = float(self.get_parameter("avoid_turn_cmd").value)
        self.avoid_drive_fwd_cmd = float(self.get_parameter("avoid_drive_fwd_cmd").value)
        self.avoid_drive_turn_cmd = float(self.get_parameter("avoid_drive_turn_cmd").value)
        self.clear_hysteresis_m = float(self.get_parameter("clear_hysteresis_m").value)
        self.scan_stale_s = float(self.get_parameter("scan_stale_s").value)
        self.max_rejoin_waypoints = int(self.get_parameter("max_rejoin_waypoints").value)
        self.avoid_timeout_s = float(self.get_parameter("avoid_timeout_s").value)
        self.avoid_min_turn_time_s = float(self.get_parameter("avoid_min_turn_time_s").value)
        self.avoid_cooldown_s = float(self.get_parameter("avoid_cooldown_s").value)
        self.avoid_commit_fwd_s = float(self.get_parameter("avoid_commit_fwd_s").value)
        self.avoid_commit_fwd_cmd = float(self.get_parameter("avoid_commit_fwd_cmd").value)
        self.bearing_free_margin_m = float(self.get_parameter("bearing_free_margin_m").value)
        self.bearing_max_abs_rad = math.radians(float(self.get_parameter("bearing_max_abs_deg").value))

        self.stall_timeout_s = float(self.get_parameter("stall_timeout_s").value)
        self.progress_eps_m = float(self.get_parameter("progress_eps_m").value)
        self.replan_cooldown_s = float(self.get_parameter("replan_cooldown_s").value)
        self.max_replans_before_next = int(self.get_parameter("max_replans_before_next").value)
        self.frontier_next_timeout_s = float(self.get_parameter("frontier_next_timeout_s").value)

        self.declare_parameter("side_choice_margin_m", 0.25)
        self.side_choice_margin_m = float(self.get_parameter("side_choice_margin_m").value)

        # ---------------- Pubs ----------------
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        self.replan_pub = self.create_publisher(Bool, self.replan_topic, 10)
        self.frontier_next_pub = self.create_publisher(Bool, self.frontier_next_topic, 10)

        # ---------------- Subs ----------------
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 50)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data)

        #qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

        #self.scan_sub = self.create_subscription(LaserScan, 'scan', self.listener_callback, qos_profile=qos_policy)

        self.path_sub = self.create_subscription(Path, self.path_topic, self.on_path, 10)
        self.goal_sub = self.create_subscription(PoseStamped, self.frontier_goal_topic, self.on_goal, 10)

        # ---------------- State ----------------
        self.have_pose = False
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # radians

        self.have_scan = False
        self.last_scan_time = 0.0
        self.scan_angle_min = 0.0
        self.scan_angle_inc = 0.0
        self.scan_range_max = 12.0
        self.scan_ranges: List[float] = []

        self.have_path = False
        self.waypoints: List[Tuple[float, float]] = []
        self.waypoint_index = 0

        self.current_frontier_goal: Optional[Tuple[float, float]] = None

        # Avoidance
        self.avoid_state = self.FOLLOW_PATH
        self.avoid_dir = 0
        self.avoid_started_t = 0.0
        self.avoid_flipped = False
        self.avoid_cooldown_until = 0.0
        self.commit_until = 0.0

        # Smoothing / rate limiting (normalized)
        self.prev_forward = 0.0
        self.prev_turn = 0.0
        self.prev_time = time.time()

        # Progress / replanning
        self.stuck_started_t = time.time()
        self.best_goal_dist = float("inf")
        self.last_progress_t = time.time()
        self.last_replan_t = 0.0
        self.replan_count = 0
        self._last_info_log_t = 0.0

        # Timer
        period = 1.0 / max(1e-6, self.control_rate_hz)
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(
            f"ExplorePathFollower (Jazzy) ready. Sub: {self.path_topic} odom:{self.odom_topic} scan:{self.scan_topic} "
            f"goal:{self.frontier_goal_topic}. Pub: {self.cmd_vel_topic} replan:{self.replan_topic} next:{self.frontier_next_topic} "
            f"limits: lin<= {self.max_linear_mps:.2f} m/s, |ang|<= {self.max_angular_radps:.2f} rad/s"
        )

    # ---------------- Callbacks ----------------
    def on_odom(self, msg: Odometry):
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        self.theta = yaw_from_quat(msg.pose.pose.orientation)  # radians
        self.have_pose = True

    def on_scan(self, msg: LaserScan):
        self.have_scan = True
        self.last_scan_time = time.time()
        self.scan_angle_min = float(msg.angle_min)
        self.scan_angle_inc = float(msg.angle_increment)
        self.scan_range_max = float(msg.range_max)
        self.scan_ranges = list(msg.ranges)



    def on_goal(self, msg: PoseStamped):
        gx = float(msg.pose.position.x)
        gy = float(msg.pose.position.y)
        self.current_frontier_goal = (gx, gy)

        # Start (or restart) deadline timer for this goal
        self.goal_received_t = time.time()
        self.goal_timed_out = False

        self.get_logger().warn(
            f"[GOAL] Frontier goal: ({gx:.2f}, {gy:.2f}) deadline={self.frontier_goal_timeout_s:.1f}s"
        )

    def on_path(self, msg: Path):
        if msg is None or len(msg.poses) < 2:
            self.get_logger().warn("[PATH] Received empty/short path. Ignoring.")
            return

        self.waypoints = [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]
        self.waypoint_index = 0
        self.have_path = True

        # Reset progress + replanning counters for this path
        self.stuck_started_t = time.time()
        self.best_goal_dist = float("inf")
        self.last_progress_t = time.time()
        self.replan_count = 0

        # Reset avoidance
        self.avoid_state = self.FOLLOW_PATH
        self.avoid_dir = 0
        self.avoid_flipped = False
        self.commit_until = 0.0
        self.avoid_cooldown_until = 0.0

        endx, endy = self.waypoints[-1]
        if self.current_frontier_goal is not None:
            gx, gy = self.current_frontier_goal
            self.get_logger().warn(
                f"[PATH] New path {len(self.waypoints)} wps. PathEnd=({endx:.2f},{endy:.2f}) "
                f"FrontierGoal=({gx:.2f},{gy:.2f}) end_to_goal={math.hypot(endx-gx, endy-gy):.2f}m"
            )
        else:
            self.get_logger().warn(f"[PATH] New path {len(self.waypoints)} wps. PathEnd=({endx:.2f},{endy:.2f})")

    # ---------------- Helpers ----------------
    def publish_cmd(self, forward_norm: float, turn_norm: float):
        """
        forward_norm: internal normalized [-1..1] but we clamp to [0..1] (no reverse)
        turn_norm: internal normalized [-1..1]
        Publishes TwistStamped with TurtleBot limits:
          linear.x: 0..0.26 m/s
          angular.z: -1.82..+1.82 rad/s
        """
        f = clamp(forward_norm, 0.0, 1.0)
        w = clamp(turn_norm, -1.0, 1.0)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""  # usually empty for cmd_vel

        msg.twist.linear.x = f * self.max_linear_mps
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = w * self.max_angular_radps

        self.cmd_pub.publish(msg)

    def publish_stop(self, repeats: int = 6):
        msg = TwistStamped()
        msg.header.frame_id = ""
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        for _ in range(max(1, repeats)):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(msg)

    def _scan_is_fresh(self) -> bool:
        return self.have_scan and ((time.time() - self.last_scan_time) <= self.scan_stale_s) and len(self.scan_ranges) > 0

    def _range_at_angle(self, ang: float) -> float:
        if not self._scan_is_fresh():
            return float("inf")

        n = len(self.scan_ranges)
        inc = self.scan_angle_inc if abs(self.scan_angle_inc) > 1e-12 else 1e-12

        a = ang % (2.0 * math.pi)
        a0 = self.scan_angle_min % (2.0 * math.pi)
        d = (a - a0) % (2.0 * math.pi)
        idx = int(round(d / inc)) % n

        r = self.scan_ranges[idx]
        if r is None or not math.isfinite(r) or r <= 0.0:
            return self.scan_range_max
        return float(r)

    def _sector_stat(self, center: float, half_width: float, mode: str = "min") -> float:
        if not self._scan_is_fresh():
            return float("inf") if mode == "min" else 0.0

        inc = self.scan_angle_inc if abs(self.scan_angle_inc) > 1e-12 else 1e-12
        span = max(inc, 2.0 * half_width)
        steps = int(max(5, min(80, span / inc)))
        if steps <= 0:
            steps = 5

        vals = []
        for i in range(-steps, steps + 1):
            a = center + (i / steps) * half_width
            r = self._range_at_angle(a)
            r = min(r, self.scan_range_max)
            vals.append(r)

        if not vals:
            return float("inf") if mode == "min" else 0.0
        return min(vals) if mode == "min" else (sum(vals) / float(len(vals)))

    def _front_blocked(self) -> bool:
        return self._sector_stat(center=0.0, half_width=self.front_cone_rad, mode="min") < self.safety_distance_m

    def _choose_avoid_direction(self) -> int:
        left_mean = self._sector_stat(center=math.pi / 2.0, half_width=self.side_cone_rad, mode="mean")
        right_mean = self._sector_stat(center=-math.pi / 2.0, half_width=self.side_cone_rad, mode="mean")

        # Keep previous direction if it's close (avoid flip-flop)
        if self.avoid_dir != 0 and abs(left_mean - right_mean) < self.side_choice_margin_m:
            dir_ = self.avoid_dir
        else:
            dir_ = +1 if left_mean >= right_mean else -1

        self.get_logger().warn(
            f"[AVOID] Side choice: left_mean={left_mean:.2f}m right_mean={right_mean:.2f}m => "
            f"{'LEFT' if dir_ > 0 else 'RIGHT'}"
        )
        return dir_

    def _bearing_to_point(self, px: float, py: float) -> Tuple[float, float]:
        dx = px - self.x
        dy = py - self.y
        dist = math.hypot(dx, dy)
        desired_heading = math.atan2(dy, dx)
        rel = angle_wrap(desired_heading - self.theta)
        return rel, dist

    def _waypoint_visible(self, wx: float, wy: float) -> bool:
        rel_ang, dist = self._bearing_to_point(wx, wy)
        if abs(rel_ang) > self.bearing_max_abs_rad:
            return False
        laser_r = self._range_at_angle(rel_ang)
        return laser_r > (dist + self.bearing_free_margin_m)

    def _find_rejoin_index(self) -> int:
        if not self.waypoints or self.waypoint_index >= len(self.waypoints):
            return -1

        end = min(len(self.waypoints), self.waypoint_index + self.max_rejoin_waypoints)
        best_i = -1
        best_dist = float("inf")

        for i in range(self.waypoint_index, end):
            wx, wy = self.waypoints[i]
            _, dist = self._bearing_to_point(wx, wy)
            if dist < 1e-6:
                return i
            if self._waypoint_visible(wx, wy):
                if dist < best_dist:
                    best_dist = dist
                    best_i = i

        if best_i == self.waypoint_index and (best_i + 1) < len(self.waypoints):
            best_i += 1
        return best_i

    def _exit_avoidance_to_follow(self, new_index: int, reason: str):
        self.waypoint_index = int(new_index)
        self.avoid_state = self.FOLLOW_PATH

        now = time.time()
        self.avoid_cooldown_until = now + self.avoid_cooldown_s
        self.commit_until = now + self.avoid_commit_fwd_s

        self.get_logger().warn(
            f"[AVOID] Exit -> FOLLOW (reason={reason}) wp_index={self.waypoint_index} "
            f"cooldown={self.avoid_cooldown_s:.2f}s commit={self.avoid_commit_fwd_s:.2f}s"
        )

    def _pulse_replan(self, reason: str):
        now = time.time()
        if (now - self.last_replan_t) < self.replan_cooldown_s:
            return
        self.last_replan_t = now
        self.replan_count += 1
        self.replan_pub.publish(Bool(data=True))
        self.get_logger().error(f"[REPLAN] #{self.replan_count} reason={reason} -> publishing /replan True")

        if self.replan_count >= self.max_replans_before_next:
            self._pulse_frontier_next("too_many_replans_no_progress")

    def _pulse_frontier_next(self, reason: str):
        self.frontier_next_pub.publish(Bool(data=True))
        self.get_logger().error(f"[FRONTIER_NEXT] reason={reason} -> publishing /frontier_next True")
        self.publish_stop(repeats=10)
        self.have_path = False
        self.waypoints = []
        self.waypoint_index = 0

    # ---------------- Main loop ----------------
    def control_loop(self):
        if not self.have_pose:
            return

        if not self.have_path or len(self.waypoints) < 2:
            self.publish_cmd(0.0, 0.0)
            return

        now = time.time()

        # ---- progress measured to FRONTIER GOAL if available; else to path end
        if self.current_frontier_goal is not None:
            gx, gy = self.current_frontier_goal
        else:
            gx, gy = self.waypoints[-1]

        goal_dist = math.hypot(gx - self.x, gy - self.y)

        # ---- Deadline per frontier goal: if not reached in time, go to next
        if (self.goal_received_t is not None) and (not self.goal_timed_out):
            if (now - self.goal_received_t) > self.frontier_goal_timeout_s and goal_dist > self.goal_tolerance_m:
                self.goal_timed_out = True
                self._pulse_frontier_next("frontier_goal_timeout")
                return

        # Progress check
        if goal_dist < (self.best_goal_dist - self.progress_eps_m):
            self.best_goal_dist = goal_dist
            self.last_progress_t = now

        # Overall stuck => frontier_next
        if (now - self.stuck_started_t) > self.frontier_next_timeout_s and goal_dist > self.goal_tolerance_m:
            self._pulse_frontier_next("stuck_timeout_overall")
            return

        # Stall => replan
        if (now - self.last_progress_t) > self.stall_timeout_s and goal_dist > self.goal_tolerance_m:
            self._pulse_replan("no_progress_stall")
            self.last_progress_t = now  # prevent immediate spam

        # ---- If we are already at frontier goal, finish (strong condition)
        if goal_dist <= self.goal_tolerance_m:
            self.publish_stop(repeats=10)
            self.have_path = False
            self.get_logger().info(
                f"[DONE] Reached frontier goal within {self.goal_tolerance_m:.2f}m. "
                f"goal=({gx:.2f},{gy:.2f}) d={goal_dist:.3f}"
            )
            return

        # ---- Waypoint advance
        wx, wy = self.waypoints[self.waypoint_index]
        _, dist_wp = self._bearing_to_point(wx, wy)
        is_last = (self.waypoint_index == len(self.waypoints) - 1)
        tol = self.goal_tolerance_m if is_last else self.waypoint_tolerance_m

        if dist_wp <= tol:
            self.waypoint_index += 1
            if self.waypoint_index >= len(self.waypoints):
                self.get_logger().warn(
                    f"[PATH] End of path reached but frontier goal not reached yet. "
                    f"frontier_goal=({gx:.2f},{gy:.2f}) d={goal_dist:.2f} > tol={self.goal_tolerance_m:.2f} -> replan"
                )
                self._pulse_replan("path_ended_not_at_goal")
                self.waypoint_index = max(0, len(self.waypoints) - 1)
                return
            return

        # ---- Avoidance triggers
        if self._scan_is_fresh():
            blocked = self._front_blocked()

            if blocked and self.avoid_state == self.FOLLOW_PATH and now >= self.avoid_cooldown_until:
                self.avoid_dir = self._choose_avoid_direction()
                self.avoid_state = self.AVOID_TURN
                self.avoid_started_t = now
                self.avoid_flipped = False
                self.commit_until = 0.0
                self.get_logger().warn(
                    f"[AVOID] FRONT BLOCKED (<{self.safety_distance_m:.2f}m). "
                    f"Entering AVOID_TURN dir={'LEFT' if self.avoid_dir > 0 else 'RIGHT'}"
                )

            if self.avoid_state in (self.AVOID_TURN, self.AVOID_DRIVE):
                if (now - self.avoid_started_t) > (2.0 * self.avoid_timeout_s):
                    self._pulse_replan("avoidance_failed_stuck")

        # ---- Compute control (normalized)
        forward = 0.0
        turn = 0.0

        # AVOIDANCE
        if self._scan_is_fresh() and self.avoid_state != self.FOLLOW_PATH:
            front_min = self._sector_stat(center=0.0, half_width=self.front_cone_rad, mode="min")
            front_clear = front_min >= (self.safety_distance_m + self.clear_hysteresis_m)

            if self.avoid_state == self.AVOID_TURN:
                forward = 0.0
                turn = clamp(self.avoid_turn_sign * self.avoid_dir * self.avoid_turn_cmd, -1.0, 1.0)

                if front_clear and (now - self.avoid_started_t) >= self.avoid_min_turn_time_s:
                    rejoin_i = self._find_rejoin_index()
                    if rejoin_i != -1:
                        self._exit_avoidance_to_follow(rejoin_i, "turn_front_clear_waypoint_free")
                    else:
                        self.avoid_state = self.AVOID_DRIVE

            elif self.avoid_state == self.AVOID_DRIVE:
                if not front_clear:
                    self.avoid_state = self.AVOID_TURN
                    forward = 0.0
                    turn = clamp(self.avoid_turn_sign * self.avoid_dir * self.avoid_turn_cmd, -1.0, 1.0)
                else:
                    forward = clamp(self.avoid_drive_fwd_cmd, 0.0, 1.0)
                    turn = clamp(self.avoid_turn_sign * self.avoid_dir * self.avoid_drive_turn_cmd, -1.0, 1.0)
                    rejoin_i = self._find_rejoin_index()
                    if rejoin_i != -1:
                        self._exit_avoidance_to_follow(rejoin_i, "drive_waypoint_bearing_free")

            if now - self._last_info_log_t > 0.8:
                self._last_info_log_t = now
                self.get_logger().info(
                    f"[AVOID] state={self.avoid_state} dir={'L' if self.avoid_dir > 0 else 'R'} "
                    f"front_min={front_min:.2f} wp={self.waypoint_index}/{len(self.waypoints)} "
                    f"frontier_goal=({gx:.2f},{gy:.2f}) dgoal={goal_dist:.2f}"
                )

        # NORMAL FOLLOW
        else:
            wx, wy = self.waypoints[self.waypoint_index]
            dx = wx - self.x
            dy = wy - self.y
            dist = math.hypot(dx, dy)

            desired_heading = math.atan2(dy, dx)
            heading_error = angle_wrap(desired_heading - self.theta)
            if self.invert_turn:
                heading_error = -heading_error

            v_des = clamp(self.K_v * dist, 0.0, 1.0) * self.max_wheel_speed_mps
            omega_max = (2.0 * self.max_wheel_speed_mps) / max(1e-6, self.track_width_m)
            omega_des = clamp(self.K_w * heading_error, -omega_max, omega_max)

            align = 1.0 - (abs(heading_error) / max(1e-6, self.turn_in_place_threshold))
            align = clamp(align, 0.0, 1.0)
            v_des *= align

            v_r = v_des + omega_des * (self.track_width_m / 2.0)
            v_l = v_des - omega_des * (self.track_width_m / 2.0)

            u_r = clamp(v_r / self.max_wheel_speed_mps, -1.0, 1.0)
            u_l = clamp(v_l / self.max_wheel_speed_mps, -1.0, 1.0)

            forward = clamp((u_l + u_r) * 0.5, -1.0, 1.0)
            turn = clamp((u_l - u_r) * 0.5, -1.0, 1.0)

            if now < self.commit_until:
                forward = max(forward, self.avoid_commit_fwd_cmd)

            if now - self._last_info_log_t > 0.8:
                self._last_info_log_t = now
                self.get_logger().info(
                    f"[FOLLOW] wp={self.waypoint_index}/{len(self.waypoints)} dist_wp={dist:.2f} "
                    f"head_err={heading_error:.2f} fwdN={forward:.2f} turnN={turn:.2f} "
                    f"frontier_goal=({gx:.2f},{gy:.2f}) dgoal={goal_dist:.2f}"
                )

        # ---- Rate limiting (normalized)
        dt = max(1e-3, now - self.prev_time)
        df = clamp(forward - self.prev_forward, -self.max_forward_rate * dt, self.max_forward_rate * dt)
        dw = clamp(turn - self.prev_turn, -self.max_turn_rate * dt, self.max_turn_rate * dt)

        forward = self.prev_forward + df
        turn = self.prev_turn + dw

        self.prev_forward = forward
        self.prev_turn = turn
        self.prev_time = now

        self.publish_cmd(forward, turn)


def main():
    rclpy.init()
    node = PathFollowerExplore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_stop(repeats=10)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
