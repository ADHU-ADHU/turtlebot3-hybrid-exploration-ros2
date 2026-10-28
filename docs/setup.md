# Setup and running

The project source contains two `ament_python` packages and one standalone Python node. The commands below follow that structure. The convenience launcher is a repository addition; it does not change the original algorithms.

## Environment

- ROS 2 Jazzy on a supported Linux installation.
- TurtleBot3 packages and a configured robot or Gazebo simulation.
- `turtlebot3_cartographer` and its dependencies.
- `colcon`, `rosdep`, NumPy, Matplotlib, and Tk.
- A graphical session for the frontier selector's `TkAgg` plot.

Follow the [ROBOTIS setup and simulation instructions](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/) with **Jazzy** selected. Clone/install a Jazzy-compatible TurtleBot3 stack before building this controller, and source its workspace if it is installed as an overlay.

The standalone follower imports `sensor_msgs`; the two package manifests do not declare that follower's dependencies. On an Ubuntu Jazzy installation, these explicit runtime packages cover that additional dependency and the plotting modules:

```bash
sudo apt install ros-jazzy-sensor-msgs python3-numpy python3-matplotlib python3-tk
```

## Build the packages

The repository root can serve as a colcon workspace because it contains `src/`. From a directory where you keep your projects:

```bash
git clone https://github.com/ADHU-ADHU/turtlebot3-hybrid-exploration-ros2.git
cd turtlebot3-hybrid-exploration-ros2
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select frontier_explorer astar_planner
source install/setup.bash
```

Source ROS and the relevant TurtleBot3/controller workspaces in every new terminal. Use the same `ROS_DOMAIN_ID` for processes that need to communicate, including the robot. Choose the domain assigned to your own setup.

## Simulation

Set the robot model in each terminal that needs it:

```bash
export TURTLEBOT3_MODEL=waffle_pi
```

In the first terminal, launch the house world:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

In the second terminal, start Cartographer:

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
```

In a third terminal, from the repository root after sourcing its installation:

```bash
ros2 launch ./launch/exploration.launch.py use_sim_time:=true
```

The wrapper starts the planner and follower, then starts the frontier selector after a short delay. Its arguments are visible without starting the nodes:

```bash
ros2 launch ./launch/exploration.launch.py --show-args
```

The launcher's delay gives subscribers time to start, but is not a readiness handshake. The goal publisher uses volatile QoS. If an initial goal is missed, check startup order and restart the selector after its subscribers are ready.

### Separate commands

To inspect the nodes individually, use three terminals instead of the wrapper. Start the follower and planner before the selector:

```bash
# Follower terminal, from the repository root
python3 src/pathfollower_new555.py --ros-args -p use_sim_time:=true
```

```bash
# Planner terminal
ros2 run astar_planner frontier_astar_planner --ros-args -p use_sim_time:=true
```

```bash
# Frontier-selector terminal
ros2 run frontier_explorer frontier_explorer_node --ros-args -p use_sim_time:=true
```

## Physical robot

Bring up your own TurtleBot3 using its configured access details and the [Jazzy bring-up guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/). On the robot, the standard command is:

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

On the remote computer, with the correct robot model and ROS domain configured:

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

Then, from the controller repository root:

```bash
ros2 launch ./launch/exploration.launch.py use_sim_time:=false
```

The follower publishes `geometry_msgs/msg/TwistStamped`. The [Jazzy TurtleBot3 bring-up documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/) describes stamped commands as the default. Check the actual driver configuration rather than relying on the older `Twist` wording in the source docstring.

Review the [implementation notes](implementation-notes.md) before physical operation. In particular, missing/stale scan data does not cause this follower to stop, and the planner/follower do not transform odometry into the map frame.

## Interface checks

These commands inspect the ROS graph without sending movement commands:

```bash
ros2 topic list
ros2 topic info /cmd_vel --verbose
ros2 topic info /scan --verbose
ros2 topic info /map --verbose
ros2 run tf2_ros tf2_echo map base_footprint
```

Expected data sources are `/map`, `/odom`, `/scan`, and `map → base_footprint` TF. Both mapping nodes also accept incremental `/map_updates` when available. A system publishing full `/map` messages can still supply updated maps without that incremental topic.

## Selected source defaults

These are source defaults, not a newly tuned simulation/hardware configuration.

| Component | Parameter | Default |
| --- | --- | --- |
| Frontier selector | `min_cluster_size` | `15` |
| Frontier selector | `robot_diameter_m` | `0.44` |
| Frontier selector | `inflation_margin_m` | `0.05` |
| Frontier selector | `map_frame` / `base_frame` | `map` / `base_footprint` |
| Planner | `robot_diameter_m` | `0.44` |
| Planner | `inflation_margin_m` | `0.10` |
| Planner | `treat_unknown_as_occupied` | `false` |
| Planner | `smooth_mavg_window` | `7` |
| Planner | `goal_candidate_attempts` | `8` |
| Follower | `control_rate_hz` | `20.0` |
| Follower | `max_linear_mps` | `0.19` |
| Follower | `max_angular_radps` | `1.6` |
| Follower | `safety_distance_m` | `0.35` |

Parameters can be overridden through ROS arguments when running a node separately. For example:

```bash
ros2 run frontier_explorer frontier_explorer_node --ros-args \
  -p use_sim_time:=true -p min_cluster_size:=10
```

The report mentions a simulation minimum cluster size of 10, while the selector defaults to 15. `max_clusters_draw=40` caps the number of clusters plotted; it is not a maximum number of cells in a cluster.

## Verification scope

Source integrity, Python syntax, package discovery/build, and launch argument inspection can be checked without driving a robot. They do not establish navigation correctness. An end-to-end Gazebo or physical-robot run is a separate validation step, particularly for the frame and recovery limitations documented in this repository.
