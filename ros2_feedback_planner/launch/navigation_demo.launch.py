import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch  # noqa: E402
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)

from launch.event_handlers import (
    OnExecutionComplete,
    OnProcessExit,
    OnProcessIO,
    OnProcessStart,
    OnShutdown
)
from launch_ros import events
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
import lifecycle_msgs.msg
# from launch.event_handlers import OnExecution
# Complete



class Nav2Ready(launch.Event):
    pass

def on_output(event: launch.events.process.ProcessIO) -> None:
    for line in event.text.decode().splitlines():
        if 'Arm tucked' in line:
            print("Arm is tucked, Gazebo is ready!")



def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    feedback_executable = LaunchConfiguration('feedback_executable')
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value=''
    )
    feedback_executable_arg = DeclareLaunchArgument(
        'feedback_executable',
        default_value='feedback_node',
        description='Feedback executable: feedback_node or ttc_feedback_node'
    )
    planning_params = os.path.join(
        get_package_share_directory('ros2_feedback_planner'),
        'config',
        'navigation_planner_config.yaml'
    )

    tiago_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tiago_gazebo'),
                'launch',
                'tiago_gazebo.launch.py'
            )
        ),
        launch_arguments={
            'namespace': namespace,
            'is_public_sim': 'True',
            'world_name': 'plasys_house',
            'x': '-5.5',
            'y': '-3.8',
            'Y': '1.5708'
        }.items()
    )

    wait_10_seconds_process = ExecuteProcess(
        cmd=['sleep', '30'],
        output='screen'
    )

    tiago_nav_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tiago_2dnav'),
                'launch',
                'tiago_nav_bringup.launch.py'
            )
        ),
        launch_arguments={
            'is_public_sim': 'True',
            'world_name': 'plasys_house'
        }.items(),
    )

    metrics_manager_node = Node(
        package='ros2_feedback_planner',
        executable='metrics_manager_node',
        name='metrics_manager_node',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[planning_params]
    )

    planning_node = Node(
        package='ros2_feedback_planner',
        executable='planner_node',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[planning_params]
    )

    feedback_node = Node(
        package='ros2_feedback_planner',
        executable=feedback_executable,
        name='feedback_node',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[planning_params]
    )

    # configure_feedback = launch.actions.EmitEvent(
    #     event=events.lifecycle.ChangeState(
    #         lifecycle_node_matcher=feedback_node,
    #         transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
    #     )
    # )

    # on_execution_complete_handler = RegisterEventHandler(
    #     OnExecutionComplete(
    #         target_action=configure_feedback,
    #         on_completion=[LogInfo(msg='Feedback configure node execution complete.')]
    #     )
    # )

    # on_process_exit_handler = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=configure_feedback,
    #         on_exit=[LogInfo(msg='Feedback node process exited.')]
    #     )
    # )

    return LaunchDescription([
        namespace_launch_arg,
        feedback_executable_arg,
        # tiago_gazebo_launch,
        tiago_nav_bringup_launch,
        metrics_manager_node,
        feedback_node,
        planning_node
        # configure_feedback,
        # on_execution_complete_handler,
        # on_process_exit_handler
    ])
