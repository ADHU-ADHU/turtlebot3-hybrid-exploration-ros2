"""Launch the submitted exploration nodes after robot and SLAM bring-up.

This portfolio convenience wrapper runs the three implementations supplied in
the submitted source archive without modifying their algorithms.
"""

from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start planner and follower before allowing frontier goal publication."""
    repo_root = Path(__file__).resolve().parents[1]
    follower_script = repo_root / 'src' / 'pathfollower_new555.py'

    use_sim_time = LaunchConfiguration('use_sim_time')
    startup_delay_s = LaunchConfiguration('startup_delay_s')
    clock_parameter = {
        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
    }

    planner = Node(
        package='astar_planner',
        executable='frontier_astar_planner',
        parameters=[clock_parameter],
        output='screen',
        on_exit=[Shutdown(reason='A* planner exited')],
    )

    follower = ExecuteProcess(
        cmd=[
            sys.executable,
            str(follower_script),
            '--ros-args',
            '-p',
            ['use_sim_time:=', use_sim_time],
        ],
        output='screen',
        on_exit=[Shutdown(reason='Path follower exited')],
    )

    frontier_selector = Node(
        package='frontier_explorer',
        executable='frontier_explorer_node',
        parameters=[clock_parameter],
        output='screen',
        on_exit=[Shutdown(reason='Frontier selector exited')],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the ROS simulation clock; set true for simulation.',
        ),
        DeclareLaunchArgument(
            'startup_delay_s',
            default_value='2.0',
            description='Seconds to wait before starting the frontier selector.',
        ),
        planner,
        follower,
        TimerAction(period=startup_delay_s, actions=[frontier_selector]),
    ])
