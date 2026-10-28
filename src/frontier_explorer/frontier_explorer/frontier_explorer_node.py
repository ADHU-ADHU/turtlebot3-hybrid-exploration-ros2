#!/usr/bin/env python3
import time
import math
from collections import deque

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path, Odometry
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from matplotlib.colors import ListedColormap, BoundaryNorm

from tf2_ros import Buffer, TransformListener


# ---------- Visualization colormap ----------
cmap = ListedColormap(["black", "lightgray", "white"])
norm = BoundaryNorm([0.0, 0.25, 0.75, 1.0], cmap.N)


def yaw_from_quat(x, y, z, w) -> float:
    # yaw (Z axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def grid_from_occupancy(msg: OccupancyGrid) -> np.ndarray | None:
    w = msg.info.width
    h = msg.info.height
    data = np.asarray(msg.data, dtype=np.int16)
    if data.size != w * h:
        return None
    return data.reshape((h, w))


def neighbor_unknown_mask(unknown: np.ndarray) -> np.ndarray:
    h, w = unknown.shape
    padded = np.pad(unknown, 1, mode="constant", constant_values=False)
    nbr = np.zeros((h, w), dtype=bool)
    nbr |= padded[0:0 + h, 0:0 + w]
    nbr |= padded[0:0 + h, 1:1 + w]
    nbr |= padded[0:0 + h, 2:2 + w]
    nbr |= padded[1:1 + h, 0:0 + w]
    nbr |= padded[1:1 + h, 2:2 + w]
    nbr |= padded[2:2 + h, 0:0 + w]
    nbr |= padded[2:2 + h, 1:1 + w]
    nbr |= padded[2:2 + h, 2:2 + w]
    return nbr


def compute_frontier_mask(free_mask: np.ndarray, unknown_mask: np.ndarray) -> np.ndarray:
    # frontier = FREE adjacent to UNKNOWN
    return free_mask & neighbor_unknown_mask(unknown_mask)


def cluster_frontiers(frontier_mask: np.ndarray, min_cluster_size: int = 20):
    h, w = frontier_mask.shape
    visited = np.zeros_like(frontier_mask, dtype=bool)
    clusters = []

    nbrs = [(-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)]

    frontier_cells = np.argwhere(frontier_mask)
    for r0, c0 in frontier_cells:
        r0, c0 = int(r0), int(c0)
        if visited[r0, c0]:
            continue

        q = deque([(r0, c0)])
        visited[r0, c0] = True
        cluster = []

        while q:
            r, c = q.popleft()
            cluster.append((r, c))
            for dr, dc in nbrs:
                rr, cc = r + dr, c + dc
                if 0 <= rr < h and 0 <= cc < w and frontier_mask[rr, cc] and not visited[rr, cc]:
                    visited[rr, cc] = True
                    q.append((rr, cc))

        if len(cluster) >= min_cluster_size:
            clusters.append(cluster)

    return clusters


def cell_to_world(r, c, origin_x, origin_y, res):
    x = origin_x + (c + 0.5) * res
    y = origin_y + (r + 0.5) * res
    return x, y


def inflate_occupied_mask(occ: np.ndarray, radius_cells: int) -> np.ndarray:
    if radius_cells <= 0:
        return occ.copy()

    h, w = occ.shape
    inflated = occ.copy()

    offs = []
    r = radius_cells
    r2 = r * r
    for dr in range(-r, r + 1):
        for dc in range(-r, r + 1):
            if dr * dr + dc * dc <= r2:
                offs.append((dr, dc))

    occ_idx = np.argwhere(occ)
    for rr, cc in occ_idx:
        rr = int(rr); cc = int(cc)
        for dr, dc in offs:
            r2_ = rr + dr
            c2_ = cc + dc
            if 0 <= r2_ < h and 0 <= c2_ < w:
                inflated[r2_, c2_] = True

    return inflated


def build_map_image_from_masks(unknown: np.ndarray, free: np.ndarray, occ_inflated: np.ndarray) -> np.ndarray:
    img = np.zeros(unknown.shape, dtype=np.float32)
    img[unknown] = 0.5
    img[free] = 1.0
    #img[occ_inflated] = 0.0
    return img


def cluster_representative_world(cluster_rc, origin_x, origin_y, res):
    rs = np.array([p[0] for p in cluster_rc], dtype=np.int32)
    cs = np.array([p[1] for p in cluster_rc], dtype=np.int32)

    r_med = int(np.median(rs))
    c_med = int(np.median(cs))

    best = None
    best_d2 = 1e18
    for (r, c) in cluster_rc:
        d2 = (r - r_med) ** 2 + (c - c_med) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best = (r, c)

    r_goal, c_goal = best
    return cell_to_world(r_goal, c_goal, origin_x, origin_y, res), (r_goal, c_goal)


class FrontierNode(Node):
    def __init__(self):
        super().__init__("frontier_selector_plotter")

        # ---------- topics ----------
        self.map_topic = "/map"
        self.map_updates_topic = "/map_updates"
        self.odom_topic = "/odom"
        self.frontier_next_topic = "/frontier_next"
        self.goal_topic = "/frontier_goal"
        self.path_topic = "/planned_path"

        # ---------- parameters ----------
        #self.declare_parameter("use_sim_time", True)

        self.declare_parameter("min_cluster_size", 15)
        self.declare_parameter("max_clusters_draw", 40)

        self.declare_parameter("robot_diameter_m", 0.44)
        self.declare_parameter("inflation_margin_m", 0.05)
        self.declare_parameter("use_inflated_for_frontiers", True)

        self.declare_parameter("goal_reached_tol_m", 0.35)
        self.declare_parameter("publish_cooldown_s", 0.5)

        self.declare_parameter("plot_show_inflated", True)

        # --- occupancy thresholds ( Cartographer) ---
        self.declare_parameter("free_thresh", 20)     # 0..20 = free
        self.declare_parameter("occ_thresh", 60)      # 65..100 = occupied
        self.declare_parameter("treat_unknown_band_as_unknown", True)
        # if True: values (free_thresh+1 .. occ_thresh-1) are treated as unknown for frontier adjacency

        # ---------- retry looping ----------
        self.declare_parameter("max_full_cycles", 3)
        self.declare_parameter("cycle_pause_s", 1.0)

        # ---------- frames (IMPORTANT) ----------
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")  # sometimes base_link

        # ---------- processing rate ----------
        self.declare_parameter("cluster_rate_hz", 5.0)  # recompute clusters at 2Hz (not every msg)

        # ---------- load params ----------
        self.min_cluster_size = int(self.get_parameter("min_cluster_size").value)
        self.max_clusters_draw = int(self.get_parameter("max_clusters_draw").value)
        self.robot_diameter_m = float(self.get_parameter("robot_diameter_m").value)
        self.inflation_margin_m = float(self.get_parameter("inflation_margin_m").value)
        self.use_inflated_for_frontiers = bool(self.get_parameter("use_inflated_for_frontiers").value)
        self.goal_reached_tol_m = float(self.get_parameter("goal_reached_tol_m").value)
        self.publish_cooldown_s = float(self.get_parameter("publish_cooldown_s").value)
        self.plot_show_inflated = bool(self.get_parameter("plot_show_inflated").value)

        self.free_thresh = int(self.get_parameter("free_thresh").value)
        self.occ_thresh = int(self.get_parameter("occ_thresh").value)
        self.treat_unknown_band_as_unknown = bool(self.get_parameter("treat_unknown_band_as_unknown").value)

        self.max_full_cycles = int(self.get_parameter("max_full_cycles").value)
        self.cycle_pause_s = float(self.get_parameter("cycle_pause_s").value)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        cluster_rate_hz = float(self.get_parameter("cluster_rate_hz").value)
        self.cluster_period = 1.0 / max(0.1, cluster_rate_hz)

        # ---------- TF ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------- state ----------
        self.map_msg: OccupancyGrid | None = None
        self.grid: np.ndarray | None = None
        self.grid_for_frontiers: np.ndarray | None = None

        # robot pose in MAP frame:
        self.robot_xy_yaw = None  # (x,y,yaw_rad)

        # fallback odom (not ideal without TF):
        self.last_odom = None  # (x,y,yaw_rad) in odom frame

        self.clusters = []
        self.cluster_goals_world = []
        self.ordered_indices = []
        self.selected_rank = 0

        self.inflated_occ = None

        self.map_count = 0
        self.current_goal_world = None
        self.current_goal_idx = None
        self.need_new_goal = True
        self.last_publish_t = 0.0
        self.explored_logged = False

        self.cycle_count = 0
        self.last_cycle_wrap_t = 0.0
        self.exploration_done = False

        self.planned_path: Path | None = None

        self.map_dirty = False  # set True by /map or /map_updates

        # ---------- QoS ----------
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL  # latched map

        upd_qos = QoSProfile(depth=10)
        upd_qos.reliability = ReliabilityPolicy.RELIABLE
        upd_qos.durability = DurabilityPolicy.VOLATILE

        # ---------- subs/pubs ----------
        self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, map_qos)
        self.create_subscription(OccupancyGridUpdate, self.map_updates_topic, self.on_map_update, upd_qos)

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 50)

        self.create_subscription(Bool, self.frontier_next_topic, self.on_frontier_next, 10)
        self.create_subscription(Path, self.path_topic, self.on_path, 10)

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)

        # timer: update pose (TF), recompute clusters (rate-limited), goal logic
        self.create_timer(self.cluster_period, self.on_timer)

        self.get_logger().info(
            f"Map: {self.map_topic} + {self.map_updates_topic} | Pose: TF({self.map_frame}->{self.base_frame}) "
            f"fallback {self.odom_topic} | Goals: {self.goal_topic}"
        )

    def on_path(self, msg: Path):
        self.planned_path = msg

    def on_frontier_next(self, msg: Bool):
        if msg.data:
            self.get_logger().warn("[FRONTIER] /frontier_next received -> skipping to next frontier.")
            self._advance_rank(reason="frontier_next", success=False)
            self.current_goal_world = None
            self.current_goal_idx = None

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.last_odom = (float(p.x), float(p.y), yaw)

    def _update_robot_pose_map(self):
        # Preferred: TF gives pose in MAP frame (consistent with /map)
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            tr = t.transform.translation
            rq = t.transform.rotation
            yaw = yaw_from_quat(rq.x, rq.y, rq.z, rq.w)
            self.robot_xy_yaw = (float(tr.x), float(tr.y), yaw)
            return
        except Exception:
            # fallback (NOT map-consistent) if TF missing
            self.robot_xy_yaw = None

    def on_map(self, msg: OccupancyGrid):
        self.map_msg = msg
        g = grid_from_occupancy(msg)
        if g is None:
            return
        self.grid = g
        self.map_dirty = True
        self.map_count += 1
        if self.map_count % 10 == 0:
            self.get_logger().info(f"/map received {self.map_count} times")

    def on_map_update(self, msg: OccupancyGridUpdate):
        # Patch the stored grid; requires that we already received /map once
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
            return

        # bounds check
        if x0 < 0 or y0 < 0 or x0 + uw > w or y0 + uh > h:
            return

        patch = data.reshape((uh, uw))
        # NOTE: grid is shaped (h, w) where row=y, col=x
        self.grid[y0:y0 + uh, x0:x0 + uw] = patch
        self.map_dirty = True

    def _compute_masks(self, grid: np.ndarray):
        unknown = (grid == -1)
        free = (grid >= 0) & (grid <= self.free_thresh)
        occ = (grid >= self.occ_thresh)

        if self.treat_unknown_band_as_unknown:
            mid_unknown = (grid > self.free_thresh) & (grid < self.occ_thresh)
            unknown = unknown | mid_unknown

        return free, unknown, occ

    def _rebuild_frontiers_and_clusters(self):
        if self.map_msg is None or self.grid is None:
            return

        info = self.map_msg.info
        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
        res = float(info.resolution)

        free, unknown, occ = self._compute_masks(self.grid)

        inflation_radius_m = (self.robot_diameter_m * 0.5) + self.inflation_margin_m
        inflation_cells = int(math.ceil(inflation_radius_m / max(1e-9, res)))
        self.inflated_occ = inflate_occupied_mask(occ, inflation_cells)

        # safe grid for validity checks
        self.grid_for_frontiers = self.grid.copy()
        self.grid_for_frontiers[self.inflated_occ] = 100

        # build masks for frontier detection (inflated or raw)
        grid_used = self.grid_for_frontiers if self.use_inflated_for_frontiers else self.grid
        free_u, unknown_u, _occ_u = self._compute_masks(grid_used)

        frontier = compute_frontier_mask(free_u, unknown_u)
        self.clusters = cluster_frontiers(frontier, min_cluster_size=self.min_cluster_size)

        # cluster representative goals
        self.cluster_goals_world = []
        for cl in self.clusters:
            (x, y), (r, c) = cluster_representative_world(cl, ox, oy, res)
            self.cluster_goals_world.append(((x, y), (r, c), len(cl)))

        self._rank_clusters()

        # If current goal disappeared, force reselection
        if self.current_goal_world is not None and self.robot_xy_yaw is not None:
            if not self._goal_still_valid():
                self.get_logger().warn("[FRONTIER] Current goal no longer valid -> reselection.")
                self.need_new_goal = True
                self.current_goal_world = None
                self.current_goal_idx = None

    def _goal_still_valid(self) -> bool:
        if self.current_goal_world is None or self.map_msg is None or self.grid_for_frontiers is None:
            return False

        info = self.map_msg.info
        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
        res = float(info.resolution)
        h, w = self.grid_for_frontiers.shape

        gx, gy = self.current_goal_world
        c = int((gx - ox) / res)
        r = int((gy - oy) / res)
        if r < 0 or r >= h or c < 0 or c >= w:
            return False

        free, unknown, occ = self._compute_masks(self.grid_for_frontiers)
        return free[r, c] and (not occ[r, c]) and (not unknown[r, c])

    def _rank_clusters(self):
        self.ordered_indices = []
        if self.robot_xy_yaw is None or not self.cluster_goals_world:
            return

        rx, ry, _ = self.robot_xy_yaw
        dlist = []
        for i, (xy, rc, sz) in enumerate(self.cluster_goals_world):
            x, y = xy
            d = math.hypot(x - rx, y - ry)
            dlist.append((d, -sz, i))

        dlist.sort()
        self.ordered_indices = [t[2] for t in dlist]

        # --- clamp selected_rank to new list length ---
        if self.selected_rank >= len(self.ordered_indices):
            self.get_logger().warn(
                f"[FRONTIER] Rank clamped: {self.selected_rank} -> 0 "
                f"(candidates shrank to {len(self.ordered_indices)})"
            )
            self.selected_rank = 0
            self.need_new_goal = True
            self.current_goal_world = None
            self.current_goal_idx = None

    def _mark_frontier_explored(self, reason: str):
        if not self.exploration_done:
            self.get_logger().warn(f"Frontier explored ({reason}).")
        self.exploration_done = True
        self.need_new_goal = False
        self.current_goal_world = None
        self.current_goal_idx = None

    def _advance_rank(self, reason: str, success: bool = False):
        if success:
            self.cycle_count = 0

        self.selected_rank += 1
        self.need_new_goal = True

        if self.ordered_indices:
            if self.selected_rank >= len(self.ordered_indices):
                self.get_logger().warn(
                    f"[FRONTIER] selected_rank {self.selected_rank} out of range for "
                    f"{len(self.ordered_indices)} candidates -> resetting to 0"
                )
                self.selected_rank = 0
                self.cycle_count += 1
                self.last_cycle_wrap_t = time.time()

                self.get_logger().warn(
                    f"[FRONTIER] Wrapped to rank 0 (cycle {self.cycle_count}/{self.max_full_cycles}) reason={reason}"
                )

                if self.cycle_count >= self.max_full_cycles:
                    self._mark_frontier_explored("max_cycles_reached_no_goal_succeeded")

    def _check_goal_reached(self):
        if self.current_goal_world is None or self.robot_xy_yaw is None:
            return
        gx, gy = self.current_goal_world
        rx, ry, _ = self.robot_xy_yaw
        d = math.hypot(gx - rx, gy - ry)
        if d <= self.goal_reached_tol_m:
            self.get_logger().info(f"[FRONTIER] Goal reached (d={d:.2f}m). Will publish next frontier.")
            self.current_goal_world = None
            self.current_goal_idx = None
            self._advance_rank(reason="goal_reached", success=True)

    def maybe_publish_next_goal(self):
        if self.exploration_done:
            return

        if self.cycle_count > 0 and (time.time() - self.last_cycle_wrap_t) < self.cycle_pause_s:
            return

        if self.map_msg is None:
            return

        if self.robot_xy_yaw is None:
            # TF not available yet; don’t publish goals in wrong frame
            return

        if not self.need_new_goal:
            return

        now = time.time()
        if (now - self.last_publish_t) < self.publish_cooldown_s:
            return

        if self.map_count > 0:
            if not self.ordered_indices:
                if not self.explored_logged:
                    self.get_logger().warn("Explored: no frontier detected (or none left to try).")
                    self.explored_logged = True
                return

        self.explored_logged = False

        idx = self.ordered_indices[self.selected_rank]
        (gx, gy), (gr, gc), sz = self.cluster_goals_world[idx]

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.map_frame
        goal.pose.position.x = float(gx)
        goal.pose.position.y = float(gy)
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)
        self.last_publish_t = now

        rx, ry, _ = self.robot_xy_yaw
        dist = math.hypot(gx - rx, gy - ry)

        self.get_logger().warn(
            f"[FRONTIER] Publishing goal rank={self.selected_rank}/{len(self.ordered_indices)-1} "
            f"cluster_idx={idx} size={sz} goal=({gx:.2f},{gy:.2f}) dist={dist:.2f}m "
            f"(free_thresh={self.free_thresh} occ_thresh={self.occ_thresh})"
        )

        self.current_goal_world = (gx, gy)
        self.current_goal_idx = idx
        self.need_new_goal = False

    def on_timer(self):
        # 1) update pose from TF (map frame)
        self._update_robot_pose_map()

        # 2) goal reached check
        self._check_goal_reached()

        # 3) recompute clusters only when map changed
        if self.map_dirty:
            self.map_dirty = False
            self._rebuild_frontiers_and_clusters()

        # 4) publish next goal if needed
        self.maybe_publish_next_goal()


def main():
    rclpy.init()
    node = FrontierNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    plt.ion()
    fig, ax = plt.subplots()
    plt.show(block=False)

    plot_hz = 5.0
    plot_dt = 1.0 / plot_hz
    last_plot = 0.0

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)

            if not plt.fignum_exists(fig.number):
                break

            now = time.time()
            if now - last_plot < plot_dt:
                continue
            last_plot = now

            if node.map_msg is None or node.grid is None or node.inflated_occ is None:
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
                plt.pause(0.001)
                continue

            info = node.map_msg.info
            ox = float(info.origin.position.x)
            oy = float(info.origin.position.y)
            res = float(info.resolution)
            h, w = node.grid.shape

            free, unknown, occ = node._compute_masks(node.grid)

            occ_for_plot = node.inflated_occ if node.plot_show_inflated else occ
            img = build_map_image_from_masks(unknown, free, occ_for_plot)

            ax.clear()
            extent = [ox, ox + w * res, oy, oy + h * res]
            ax.imshow(img, origin="lower", extent=extent, vmin=0.0, vmax=1.0, cmap=cmap)

            for cl in node.clusters[: node.max_clusters_draw]:
                xs = np.empty(len(cl), dtype=np.float32)
                ys = np.empty(len(cl), dtype=np.float32)
                for k, (r, c) in enumerate(cl):
                    x, y = cell_to_world(r, c, ox, oy, res)
                    xs[k] = x
                    ys[k] = y
                ax.scatter(xs, ys, s=3)

            if node.cluster_goals_world:
                gx_all = [g[0][0] for g in node.cluster_goals_world]
                gy_all = [g[0][1] for g in node.cluster_goals_world]
                ax.scatter(gx_all, gy_all, s=70, marker="x")

                for rank, idx in enumerate(node.ordered_indices[:40]):
                    (gx, gy), _, _ = node.cluster_goals_world[idx]
                    ax.text(gx, gy, str(rank), fontsize=9)

            if node.planned_path is not None and len(node.planned_path.poses) > 1:
                px = [p.pose.position.x for p in node.planned_path.poses]
                py = [p.pose.position.y for p in node.planned_path.poses]
                ax.plot(px, py, linewidth=2)

            if node.current_goal_world is not None:
                gx, gy = node.current_goal_world
                ax.scatter([gx], [gy], s=140, marker="*", linewidths=2)

            if node.robot_xy_yaw is not None:
                rx, ry, yaw = node.robot_xy_yaw
                ax.scatter([rx], [ry], s=120, marker="o")
                ax.arrow(rx, ry, 0.4 * math.cos(yaw), 0.4 * math.sin(yaw),
                         width=0.02, length_includes_head=True)

            ax.set_title(f"Frontier clusters: {len(node.clusters)} | /map msgs: {node.map_count}")
            ax.set_xlabel("x [m]")
            ax.set_ylabel("y [m]")

            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            plt.pause(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
