#!/usr/bin/env python3
import math
import time
import heapq
from dataclasses import dataclass
from typing import List, Tuple, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from map_msgs.msg import OccupancyGridUpdate


from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def yaw_from_quat(q) -> float:
    """geometry_msgs/Quaternion -> yaw (rad)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def grid_from_occupancy(msg: OccupancyGrid) -> Optional[np.ndarray]:
    w = msg.info.width
    h = msg.info.height
    data = np.asarray(msg.data, dtype=np.int16)
    if data.size != w * h:
        return None
    return data.reshape((h, w))


def inflate_occupied_mask(occ: np.ndarray, radius_cells: int) -> np.ndarray:
    """Inflate occupied cells by radius (cells) with circular stencil."""
    if radius_cells <= 0:
        return occ.copy()

    h, w = occ.shape
    inflated = occ.copy()

    r = radius_cells
    r2 = r * r
    offsets = []
    for dr in range(-r, r + 1):
        for dc in range(-r, r + 1):
            if dr * dr + dc * dc <= r2:
                offsets.append((dr, dc))

    occ_idx = np.argwhere(occ)
    for rr, cc in occ_idx:
        rr = int(rr); cc = int(cc)
        for dr, dc in offsets:
            r2_ = rr + dr
            c2_ = cc + dc
            if 0 <= r2_ < h and 0 <= c2_ < w:
                inflated[r2_, c2_] = True

    return inflated


def world_to_cell(x: float, y: float, ox: float, oy: float, res: float) -> Tuple[int, int]:
    c = int((x - ox) / res)
    r = int((y - oy) / res)
    return r, c


def cell_to_world(r: int, c: int, ox: float, oy: float, res: float) -> Tuple[float, float]:
    x = ox + (c + 0.5) * res
    y = oy + (r + 0.5) * res
    return x, y


def bresenham_cells(r0, c0, r1, c1):
    """Yield grid cells along a line (Bresenham)."""
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dr - dc

    r, c = r0, c0
    while True:
        yield r, c
        if r == r1 and c == c1:
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc


def line_of_sight_free(occ: np.ndarray, a: Tuple[int, int], b: Tuple[int, int]) -> bool:
    """True if segment a->b goes only through free cells (occ == False)."""
    (r0, c0) = a
    (r1, c1) = b
    h, w = occ.shape
    for r, c in bresenham_cells(r0, c0, r1, c1):
        if not (0 <= r < h and 0 <= c < w):
            return False
        if occ[r, c]:
            return False
    return True


def shortcut_path(path_rc: List[Tuple[int, int]], occ: np.ndarray) -> List[Tuple[int, int]]:
    """String-pulling: keep endpoints, remove unnecessary intermediate points."""
    if len(path_rc) <= 2:
        return path_rc[:]

    out = [path_rc[0]]
    i = 0
    while i < len(path_rc) - 1:
        j = len(path_rc) - 1
        while j > i + 1:
            if line_of_sight_free(occ, path_rc[i], path_rc[j]):
                break
            j -= 1
        out.append(path_rc[j])
        i = j
    return out


def moving_average_smooth(xy: List[Tuple[float, float]], window: int = 5) -> List[Tuple[float, float]]:
    if len(xy) < 3 or window <= 1:
        return xy[:]
    w = max(3, window)
    w = w if (w % 2 == 1) else (w + 1)
    k = w // 2
    xs = np.array([p[0] for p in xy], dtype=np.float32)
    ys = np.array([p[1] for p in xy], dtype=np.float32)
    xs2 = xs.copy()
    ys2 = ys.copy()
    for i in range(k, len(xy) - k):
        xs2[i] = xs[i - k:i + k + 1].mean()
        ys2[i] = ys[i - k:i + k + 1].mean()
    xs2[0], ys2[0] = xs[0], ys[0]
    xs2[-1], ys2[-1] = xs[-1], ys[-1]
    return list(zip(xs2.tolist(), ys2.tolist()))


@dataclass
class AStarResult:
    success: bool
    path_rc: List[Tuple[int, int]]


class FrontierAStarPlanner(Node):
    def __init__(self):
        super().__init__("frontier_astar_planner")

        # ---------------- params ----------------
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("pose_topic", "/odom")  # Jazzy: use odom by default
        self.declare_parameter("goal_topic", "/frontier_goal")
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("frontier_next_topic", "/frontier_next")
        self.declare_parameter("replan_topic", "/replan")

        self.declare_parameter("robot_diameter_m", 0.44)
        self.declare_parameter("inflation_margin_m", 0.1) # 0.035
        self.declare_parameter("treat_unknown_as_occupied", False)

        self.declare_parameter("max_astar_expansions", 250000)
        self.declare_parameter("goal_search_radius_cells", 20)
        self.declare_parameter("start_search_radius_cells", 20)

        self.declare_parameter("smooth_shortcut", True)
        self.declare_parameter("smooth_mavg_window", 7)

        self.declare_parameter("replan_on_map_update", True)
        self.declare_parameter("map_replan_min_period_s", 0.2)
        self.declare_parameter("max_failed_plans_before_next", 15)
        self.declare_parameter("goal_candidate_attempts", 8)

        self.declare_parameter("free_thresh", 40)
        self.declare_parameter("occ_thresh", 60)
        self.declare_parameter("treat_unknown_band_as_unknown", True)

        self.free_thresh = int(self.get_parameter("free_thresh").value)
        self.occ_thresh = int(self.get_parameter("occ_thresh").value)
        self.treat_unknown_band_as_unknown = bool(self.get_parameter("treat_unknown_band_as_unknown").value)

        self.map_topic = str(self.get_parameter("map_topic").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.frontier_next_topic = str(self.get_parameter("frontier_next_topic").value)
        self.replan_topic = str(self.get_parameter("replan_topic").value)

        self.robot_diameter_m = float(self.get_parameter("robot_diameter_m").value)
        self.inflation_margin_m = float(self.get_parameter("inflation_margin_m").value)
        self.treat_unknown_as_occupied = bool(self.get_parameter("treat_unknown_as_occupied").value)

        self.max_astar_expansions = int(self.get_parameter("max_astar_expansions").value)
        self.goal_search_radius_cells = int(self.get_parameter("goal_search_radius_cells").value)
        self.start_search_radius_cells = int(self.get_parameter("start_search_radius_cells").value)

        self.smooth_shortcut = bool(self.get_parameter("smooth_shortcut").value)
        self.smooth_mavg_window = int(self.get_parameter("smooth_mavg_window").value)

        self.replan_on_map_update = bool(self.get_parameter("replan_on_map_update").value)
        self.map_replan_min_period_s = float(self.get_parameter("map_replan_min_period_s").value)
        self.max_failed_plans_before_next = int(self.get_parameter("max_failed_plans_before_next").value)
        self.goal_candidate_attempts = int(self.get_parameter("goal_candidate_attempts").value)

        self.failed_plan_count = 0
        self._last_plan_t = 0.0

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        upd_qos = QoSProfile(depth=10)
        upd_qos.reliability = ReliabilityPolicy.RELIABLE
        upd_qos.durability = DurabilityPolicy.VOLATILE

        self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, map_qos)
        self.create_subscription(OccupancyGridUpdate, "/map_updates", self.on_map_update, upd_qos)

        # ---------------- state ----------------
        self.map_msg: OccupancyGrid | None = None
        self.grid: np.ndarray | None = None
        self.occ_inflated: np.ndarray | None = None

        # Jazzy: pose from Odometry -> store (x, y, yaw_rad)
        self.pose_xyyaw: Optional[Tuple[float, float, float]] = None
        self.have_pose = False

        self.current_goal: PoseStamped | None = None
        self.have_goal = False

        self._last_fail_pulse_t = 0.0

        # ---------------- qos + pubs/subs ----------------
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, map_qos)
        self.create_subscription(Odometry, self.pose_topic, self.on_odom, 50)
        self.create_subscription(PoseStamped, self.goal_topic, self.on_goal, 10)
        self.create_subscription(Bool, self.replan_topic, self.on_replan, 10)

        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.frontier_next_pub = self.create_publisher(Bool, self.frontier_next_topic, 10)

        self.get_logger().info(
            f"A* planner ready. Sub: {self.goal_topic}, {self.replan_topic}, pose:{self.pose_topic}. "
            f"Pub: {self.path_topic}, {self.frontier_next_topic}"
        )

    # ---------- callbacks ----------
    def on_odom(self, msg: Odometry):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.pose_xyyaw = (x, y, yaw)
        self.have_pose = True

    def _compute_masks(self, grid: np.ndarray):
        unknown = (grid == -1)
        free = (grid >= 0) & (grid <= self.free_thresh)
        occ = (grid >= self.occ_thresh)

        if self.treat_unknown_band_as_unknown:
            mid_unknown = (grid > self.free_thresh) & (grid < self.occ_thresh)
            unknown = unknown | mid_unknown

        return free, unknown, occ

    def on_map(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.grid = grid_from_occupancy(msg)
        if self.grid is None:
            self.get_logger().error("Map grid parse failed.")
            return
        self._recompute_inflated_occ()

        res = float(msg.info.resolution)
        inflation_radius_m = (self.robot_diameter_m * 0.5) + self.inflation_margin_m
        inflation_cells = int(math.ceil(inflation_radius_m / max(1e-9, res)))

        free, unknown, occ = self._compute_masks(self.grid)

        # For planning you usually want to forbid unknown (and the mid band, if enabled)
        occ_for_planning = occ
        if self.treat_unknown_as_occupied:
            occ_for_planning = occ_for_planning | unknown

        self.occ_inflated = inflate_occupied_mask(occ_for_planning, inflation_cells)

        if self.replan_on_map_update and self.have_goal and self.have_pose:
            now = time.time()
            if (now - self._last_plan_t) >= self.map_replan_min_period_s:
                self.plan_and_publish(trigger="map_update")

    def _recompute_inflated_occ(self):
        """Rebuild self.occ_inflated from self.grid using _compute_masks + inflation params."""
        if self.map_msg is None or self.grid is None:
            return

        res = float(self.map_msg.info.resolution)
        inflation_radius_m = (self.robot_diameter_m * 0.5) + self.inflation_margin_m
        inflation_cells = int(math.ceil(inflation_radius_m / max(1e-9, res)))

        free, unknown, occ = self._compute_masks(self.grid)

        occ_for_planning = occ
        if self.treat_unknown_as_occupied:
            occ_for_planning = occ_for_planning | unknown

        self.occ_inflated = inflate_occupied_mask(occ_for_planning, inflation_cells)

    def on_map_update(self, msg: OccupancyGridUpdate):
        # Need an existing full map first
        if self.grid is None or self.map_msg is None:
            return

        info = self.map_msg.info
        w = int(info.width)
        h = int(info.height)

        x0 = int(msg.x)
        y0 = int(msg.y)
        uw = int(msg.width)
        uh = int(msg.height)

        data = np.asarray(msg.data, dtype=np.int16)
        if data.size != uw * uh:
            self.get_logger().warn("Map update size mismatch -> ignoring update.")
            return

        # Bounds check
        if x0 < 0 or y0 < 0 or (x0 + uw) > w or (y0 + uh) > h:
            self.get_logger().warn("Map update out of bounds -> ignoring update.")
            return

        patch = data.reshape((uh, uw))

        # grid is (h,w) where row=y, col=x
        self.grid[y0:y0 + uh, x0:x0 + uw] = patch

        # Recompute inflated occupancy from updated grid
        self._recompute_inflated_occ()

        # Optional: replan if enabled
        if self.replan_on_map_update and self.have_goal and self.have_pose:
            now = time.time()
            if (now - self._last_plan_t) >= self.map_replan_min_period_s:
                self.plan_and_publish(trigger="map_update")

    def free_goal_candidates(self, occ: np.ndarray, gr: int, gc: int, radius: int) -> List[Tuple[int, int, int]]:
        """Returns list of (d2, r, c) for FREE cells within radius, sorted by distance to (gr,gc)."""
        h, w = occ.shape
        out = []
        r0, c0 = gr, gc
        for dr in range(-radius, radius + 1):
            rr = r0 + dr
            if rr < 0 or rr >= h:
                continue
            for dc in range(-radius, radius + 1):
                cc = c0 + dc
                if cc < 0 or cc >= w:
                    continue
                if occ[rr, cc]:
                    continue
                d2 = dr * dr + dc * dc
                out.append((d2, rr, cc))
        out.sort(key=lambda t: t[0])
        return out

    def on_goal(self, msg: PoseStamped):
        self.current_goal = msg
        self.have_goal = True
        self.failed_plan_count = 0
        self.get_logger().warn(
            f"[GOAL] Received frontier goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) -> planning"
        )
        self.plan_and_publish(trigger="goal")

    def on_replan(self, msg: Bool):
        if msg.data and self.have_goal:
            self.get_logger().warn("[REPLAN] /replan True -> replanning to same goal from current pose")
            self.plan_and_publish(trigger="replan")

    # ---------- utilities ----------
    def pulse_frontier_next(self, reason: str):
        now = time.time()
        if now - self._last_fail_pulse_t < 0.5:
            return
        self._last_fail_pulse_t = now
        self.get_logger().error(f"[FAIL] {reason} -> publishing /frontier_next True")
        self.frontier_next_pub.publish(Bool(data=True))

    def record_plan_failure(self, reason: str, trigger: str):
        self.failed_plan_count += 1
        self.get_logger().error(
            f"[FAIL] {reason} fail_count={self.failed_plan_count}/{self.max_failed_plans_before_next} trigger={trigger}"
        )
        if self.failed_plan_count >= self.max_failed_plans_before_next:
            self.failed_plan_count = 0
            self.pulse_frontier_next(reason + " (max attempts reached)")
            self.have_goal = False
            self.current_goal = None

    def nearest_free(self, occ: np.ndarray, r0: int, c0: int, radius: int) -> Optional[Tuple[int, int]]:
        h, w = occ.shape
        if 0 <= r0 < h and 0 <= c0 < w and not occ[r0, c0]:
            return (r0, c0)

        best = None
        best_d2 = 1e18
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                rr = r0 + dr
                cc = c0 + dc
                if 0 <= rr < h and 0 <= cc < w and not occ[rr, cc]:
                    d2 = dr * dr + dc * dc
                    if d2 < best_d2:
                        best_d2 = d2
                        best = (rr, cc)
        return best

    # ---------- A* ----------
    def astar(self, occ: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int]) -> AStarResult:
        h, w = occ.shape
        (sr, sc) = start
        (gr, gc) = goal

        if occ[sr, sc] or occ[gr, gc]:
            return AStarResult(False, [])

        moves = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))
        ]

        def hcost(r, c):
            return math.hypot(r - gr, c - gc)

        gscore = np.full((h, w), np.inf, dtype=np.float32)
        gscore[sr, sc] = 0.0

        came_from_r = np.full((h, w), -1, dtype=np.int32)
        came_from_c = np.full((h, w), -1, dtype=np.int32)

        open_heap = []
        heapq.heappush(open_heap, (hcost(sr, sc), 0.0, sr, sc))

        expansions = 0

        # Track the best (closest-to-goal) node we actually pop/expand
        best_r, best_c = sr, sc
        best_h = hcost(sr, sc)

        while open_heap:
            f, g, r, c = heapq.heappop(open_heap)

            # Update best expanded node
            hc = hcost(r, c)
            if hc < best_h:
                best_h = hc
                best_r, best_c = r, c

            if (r, c) == (gr, gc):
                # Reconstruct full path
                path = [(r, c)]
                while not (r == sr and c == sc):
                    pr = int(came_from_r[r, c])
                    pc = int(came_from_c[r, c])
                    if pr < 0:
                        break
                    r, c = pr, pc
                    path.append((r, c))
                path.reverse()
                return AStarResult(True, path)

            expansions += 1
            if expansions > self.max_astar_expansions:
                break

            for dr, dc, cost in moves:
                rr = r + dr
                cc = c + dc
                if rr < 0 or rr >= h or cc < 0 or cc >= w:
                    continue
                if occ[rr, cc]:
                    continue

                ng = g + cost
                if ng < gscore[rr, cc]:
                    gscore[rr, cc] = ng
                    came_from_r[rr, cc] = r
                    came_from_c[rr, cc] = c
                    nf = ng + hcost(rr, cc)
                    heapq.heappush(open_heap, (nf, ng, rr, cc))

        # Goal not reached: reconstruct best-effort path to best_r,best_c
        if (best_r, best_c) == (sr, sc):
            return AStarResult(False, [(sr, sc)])

        r, c = best_r, best_c
        path = [(r, c)]
        while not (r == sr and c == sc):
            pr = int(came_from_r[r, c])
            pc = int(came_from_c[r, c])
            if pr < 0:
                break
            r, c = pr, pc
            path.append((r, c))
        path.reverse()
        return AStarResult(False, path)

    def plan_and_publish(self, trigger: str = "goal"):
        self._last_plan_t = time.time()

        if self.map_msg is None or self.grid is None or self.occ_inflated is None:
            self.get_logger().warn("No map yet -> cannot plan.")
            return
        if not self.have_pose or self.pose_xyyaw is None:
            self.get_logger().warn("No pose yet -> cannot plan.")
            return
        if not self.have_goal or self.current_goal is None:
            self.get_logger().warn("No goal yet -> cannot plan.")
            return

        self._recompute_inflated_occ()
        if self.occ_inflated is None:
            self.get_logger().warn("No inflated occupancy -> cannot plan.")
            return
        occ = self.occ_inflated.copy()

        info = self.map_msg.info
        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
        res = float(info.resolution)
        h, w = self.grid.shape

        sx, sy, _ = self.pose_xyyaw
        gx = float(self.current_goal.pose.position.x)
        gy = float(self.current_goal.pose.position.y)

        sr, sc = world_to_cell(sx, sy, ox, oy, res)
        gr, gc = world_to_cell(gx, gy, ox, oy, res)

        if not (0 <= sr < h and 0 <= sc < w):
            self.record_plan_failure("Start out of map bounds", trigger)
            return
        if not (0 <= gr < h and 0 <= gc < w):
            self.record_plan_failure("Goal out of map bounds", trigger)
            return

        #start_free = self.nearest_free(self.occ_inflated, sr, sc, self.start_search_radius_cells)
        start_free = self.nearest_free(occ, sr, sc, self.start_search_radius_cells)

        if start_free is None:
            self.record_plan_failure("No free start cell nearby", trigger)
            return

        (sr2, sc2) = start_free


        #cands = self.free_goal_candidates(self.occ_inflated, gr, gc, self.goal_search_radius_cells)
        cands = self.free_goal_candidates(occ, gr, gc, self.goal_search_radius_cells)

        if not cands:
            self.record_plan_failure("No free goal cell nearby (inflated map)", trigger)
            return

        attempts = min(self.goal_candidate_attempts, len(cands))
        #raw_occ = bool(self.occ_inflated[gr, gc]) if (0 <= gr < h and 0 <= gc < w) else True
        raw_occ = bool(occ[gr, gc]) if (0 <= gr < h and 0 <= gc < w) else True

        best_path_rc = None
        best_gr2 = None
        best_gc2 = None
        best_time_ms = None

        # NEW: keep best partial too
        best_partial_rc = None
        best_partial_rem = float("inf")  # remaining distance (cells) to that candidate
        best_partial_time_ms = None
        best_partial_used_goal = None  # (gr2,gc2)

        for k in range(attempts):
            _, gr2, gc2 = cands[k]
            used_occ = bool(occ[gr2, gc2])

            gx_used, gy_used = cell_to_world(gr2, gc2, ox, oy, res)
            shift = math.hypot(gx_used - gx, gy_used - gy)

            self.get_logger().warn(
                f"[GOAL_TRY {k + 1}/{attempts}] raw_goal=({gx:.2f},{gy:.2f}) raw_occ={raw_occ} | "
                f"used_goal=({gx_used:.2f},{gy_used:.2f}) used_occ={used_occ} shift={shift:.3f}m trigger={trigger}"
            )

            t0 = time.time()
            res_astar = self.astar(occ, (sr2, sc2), (gr2, gc2))
            t1 = time.time()
            dt_ms = (t1 - t0) * 1000.0

            if res_astar.path_rc and len(res_astar.path_rc) >= 2:
                end_r, end_c = res_astar.path_rc[-1]
                rem = math.hypot(end_r - gr2, end_c - gc2)  # remaining distance in cells

                # Keep the best partial (closest-to-goal endpoint)
                if rem < best_partial_rem:
                    best_partial_rem = rem
                    best_partial_rc = res_astar.path_rc
                    best_partial_time_ms = dt_ms
                    best_partial_used_goal = (gr2, gc2)

            if res_astar.success and len(res_astar.path_rc) >= 2:
                best_path_rc = res_astar.path_rc
                best_gr2, best_gc2 = gr2, gc2
                best_time_ms = dt_ms
                break

        if best_path_rc is None:
            # No full path: publish best partial if we have one
            if best_partial_rc is None or len(best_partial_rc) < 2:
                self.record_plan_failure(f"A* failed for {attempts} goal candidates (no partial)", trigger)
                return

            self.get_logger().warn(
                f"[PARTIAL] Goal unreachable -> publishing partial path. "
                f"best_remaining≈{best_partial_rem:.1f} cells trigger={trigger}"
            )

            path_rc = best_partial_rc
            best_time_ms = best_partial_time_ms

            # For logging only: use the candidate goal we were aiming for
            gr2, gc2 = best_partial_used_goal
        else:
            path_rc = best_path_rc
            gr2, gc2 = best_gr2, best_gc2

        self.failed_plan_count = 0

        #gr2, gc2 = best_gr2, best_gc2
        #path_rc = best_path_rc

        gx_used, gy_used = cell_to_world(gr2, gc2, ox, oy, res)

        if self.smooth_shortcut:
            #path_rc = shortcut_path(path_rc, self.occ_inflated)
            path_rc = shortcut_path(path_rc, occ)

        xy = [cell_to_world(r, c, ox, oy, res) for (r, c) in path_rc]
        xy = moving_average_smooth(xy, window=self.smooth_mavg_window)

        self.get_logger().warn(
            f"[PATH_END] raw_goal=({gx:.2f},{gy:.2f}) "
            f"used_goal=({gx_used:.2f},{gy_used:.2f}) "
            f"path_end=({xy[-1][0]:.2f},{xy[-1][1]:.2f}) "
            f"d(end,used)={math.hypot(xy[-1][0] - gx_used, xy[-1][1] - gy_used):.3f} "
            f"d(end,raw)={math.hypot(xy[-1][0] - gx, xy[-1][1] - gy):.3f}"
        )

        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"

        poses = []
        for (x, y) in xy:
            ps = PoseStamped()
            ps.header = out.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            poses.append(ps)

        out.poses = poses
        self.path_pub.publish(out)

        length = 0.0
        for i in range(1, len(xy)):
            length += math.hypot(xy[i][0] - xy[i - 1][0], xy[i][1] - xy[i - 1][1])

        self.get_logger().info(
            f"[PLAN] success: points={len(xy)} raw_points={len(best_path_rc)} "
            f"time={best_time_ms:.1f}ms length={length:.2f}m "
            f"start=({sr2},{sc2}) goal=({gr2},{gc2}) trigger={trigger}"
        )


def main():
    rclpy.init()
    node = FrontierAStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

