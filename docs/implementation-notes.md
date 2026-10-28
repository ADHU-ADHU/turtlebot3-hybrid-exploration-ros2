# Implementation notes

The repository preserves the original source of the exploration controller. The report describes the observed simulation and laboratory behavior. Source review also identifies limitations that matter when reproducing or extending the implementation.

## Coordinate frames

The frontier selector looks up `map → base_footprint` using TF. The planner and follower instead take raw `/odom` positions and compare them directly with map-frame goals and paths. They do not transform those positions into `map`.

This assumes the coordinate frames coincide. SLAM corrections or a non-identity `map → odom` transform can invalidate that assumption. A future revision should obtain or transform all planning and tracking positions in one consistent frame.

References: [selector pose lookup](../src/frontier_explorer/frontier_explorer/frontier_explorer_node.py#L308-L319), [planner pose input](../src/astar_planner/astar_planner/frontier_astar_planner.py#L264-L269), [planner start conversion](../src/astar_planner/astar_planner/frontier_astar_planner.py#L550-L555), [follower odometry](../src/pathfollower_new555.py#L243-L247).

## Partial-path fallback

The A* planner supports returning a best-effort partial route when the goal is unreachable. In that branch, `best_path_rc` remains `None`. The planner publishes the selected partial path, then its logging statement evaluates `len(best_path_rc)`, raising `TypeError`.

The original code is retained. Fixing this requires logging the selected route and distinguishing partial from complete planning success.

Reference: [partial-path selection and publication](../src/astar_planner/astar_planner/frontier_astar_planner.py#L630-L696).

## Path and obstacle assumptions

- Unknown space is traversable by default because `treat_unknown_as_occupied` is `false`.
- A* permits diagonal moves without checking both adjoining orthogonal cells for corner crossing.
- Moving-average smoothing is applied after line-of-sight shortcutting, without a final collision check on the adjusted path.
- The selector and planner use different occupancy thresholds and inflation margins. Their default inflation radii are 0.27 m and 0.32 m respectively.

These choices mean that obstacle inflation and smoothing do not provide a general guarantee of collision-free execution. A future revision should make the policy explicit and validate the final trajectory against the map.

References: [planner defaults](../src/astar_planner/astar_planner/frontier_astar_planner.py#L170-L188), [A* expansion](../src/astar_planner/astar_planner/frontier_astar_planner.py#L493-L507), [smoothing](../src/astar_planner/astar_planner/frontier_astar_planner.py#L657-L662).

## Reactive control and recovery

The follower uses `qos_profile_sensor_data` for LiDAR and computes beam directions from scan geometry. However, a missing or stale scan bypasses reactive avoidance and allows ordinary path tracking; there is no stop-on-stale-scan behavior.

At the replan retry limit, a recovery helper can clear `waypoints` while the calling control loop continues and subsequently indexes that list. This can raise `IndexError`. Incoming replacement paths also reset progress and recovery counters, which interacts with frequent map-triggered replanning.

References: [recovery helpers](../src/pathfollower_new555.py#L433-L467), [control loop](../src/pathfollower_new555.py#L470-L649), [path update resets](../src/pathfollower_new555.py#L272-L292).

## Timing and visualization

The nodes accept ROS `use_sim_time`, but many freshness checks, cooldowns, and deadlines use wall time through `time.time()`. Pausing or changing simulation speed therefore does not uniformly pause all controller timing.

The frontier selector forces the `TkAgg` backend and requires a graphical display. Closing its figure stops the node. Its inflated-obstacle display option is not fully implemented in the original drawing helper.

The planner also contains duplicate map subscriptions and repeated inflation work. These are opportunities for cleanup rather than reported benchmark findings.

## Results and evaluation

The report provides qualitative simulation and hardware observations, together with intermediate map/path plots. The physical-robot figure shows partial laboratory mapping. Full laboratory coverage, measured collision-free reliability, or quantitative performance improvements are not established by these figures.

Reported real-world challenges include LiDAR noise around reflective and transparent materials, localization drift, and occasional map collapse. Useful next steps are repeatable Gazebo scenarios and measurements of coverage, exploration time, route length, recovery frequency, and collision counts.

## Packaging and licensing metadata

The package metadata contains template maintainer fields, version differences between `setup.py` and `package.xml`, and inconsistent license descriptions. The manifests declare `Apache-2.0`, while `setup.py` retains a license placeholder. No repository-wide license has been added to resolve that ambiguity on the authors' behalf.

The included package tests are ROS template checks rather than navigation or algorithm tests. The standalone follower is run as a script, not installed as a package entry point.
