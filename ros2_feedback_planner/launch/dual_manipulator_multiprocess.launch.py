"""Launch file for dual manipulator data generation with multi-process architecture."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description."""
    # Get config file path
    pkg_share = get_package_share_directory('ros2_feedback_planner')
    config_file = os.path.join(
        pkg_share, 'config', 'data_generation_manipulation.yaml'
    )

    return LaunchDescription([
        # Robot 1 Controller
        Node(
            package='ros2_feedback_planner',
            executable='robot_controller',
            name='robot1_controller',
            arguments=['robot1', '--ros-args', '--log-level', 'fatal'],
            output='screen',
            emulate_tty=True,
        ),
        
        # Robot 2 Controller
        Node(
            package='ros2_feedback_planner',
            executable='robot_controller',
            name='robot2_controller',
            arguments=['robot2', '--ros-args', '--log-level', 'fatal'],
            output='screen',
            emulate_tty=True,
        ),
        
        # Scenario Coordinator
        Node(
            package='ros2_feedback_planner',
            executable='scenario_coordinator',
            name='scenario_coordinator',
            parameters=[config_file],
            arguments=['--ros-args', '--log-level', 'fatal'],
            output='screen',
            emulate_tty=True,
        ),
    ])
