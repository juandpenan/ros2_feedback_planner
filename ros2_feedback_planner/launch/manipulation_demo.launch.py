import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

 
def generate_launch_description():
    # Launch arguments
    namespace = LaunchConfiguration('namespace')
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the manipulator simulator'
    )
    planning_params = os.path.join(
        get_package_share_directory('ros2_feedback_planner'),
        'config',
        'manipulation_planner_config.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use /clock (simulation time)'
    )

    log_level = LaunchConfiguration('log_level')
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='fatal',
        description='Log level for all nodes (debug, info, warn, error, fatal)'
    )

    manipulator_simulator = Node(
        package='ros2_feedback_planner',
        executable='manipulator_simulator',
        name='manipulator_simulator',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        respawn=True,
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[planning_params,
                    # {'use_sim_time': use_sim_time},
                    ]
    )

    metrics_manager_node = Node(
        package='ros2_feedback_planner',
        executable='metrics_manager_node',
        name='metrics_manager_node',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        respawn=True,
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[planning_params]
    )

    planning_node = Node(
        package='ros2_feedback_planner',
        executable='planner_node',
        name='planner_node',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        respawn=True,
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[planning_params]
    )

    feedback_node = Node(
        package='ros2_feedback_planner',
        executable='feedback_node',
        name='feedback_node',
        respawn=True,
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[planning_params]
    )

    return LaunchDescription([
        namespace_launch_arg,
        use_sim_time_arg,
        log_level_arg,
        manipulator_simulator,
        planning_node,
        feedback_node,
        metrics_manager_node
    ])
