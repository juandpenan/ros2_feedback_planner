"""Actions for controlling robot navigation using nav2_simple_commander."""

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from moveit_configs_utils import MoveItConfigsBuilder
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.core.robot_state import RobotState
from moveit.core.planning_scene import PlanningScene
from ros2_feedback_planner.utils import get_gz_pose
from moveit.utils import create_params_file_from_dict
from ament_index_python.packages import get_package_share_directory
import time
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import ExecuteTrajectory
from action_msgs.msg import GoalStatus

class BaseAction:
    """BaseAction provides robot navigation actions using nav2_simple_commander."""

    def __init__(self, backend: 'nav2'):
        """Initialize the BaseAction with a BasicNavigator instance.

        available backends are: nav2 or moveit
        """
        if backend == 'nav2':
            self.use_nav = True
            self.use_moveit = False
        elif backend == 'moveit':
            self.use_moveit = True
            self.use_nav = False
        else:
            self.use_moveit = False
            self.use_nav = False

        if self.use_nav:
            self.all_methods = {
                'move_forward': self._move_forward,
                'move_backwards': self._move_backwards,
                'move_left': self._move_left,
                'move_right': self._move_right,
                'turn_left': self._turn_left,
                'turn_right': self._turn_right,
                'move_to': self._move_to,
                'done': self._done
            }
            self.locations = self._init_poses()
            self.navigator = BasicNavigator(node_name='navigator')
            self.navigator.waitUntilNav2Active()

        if self.use_moveit:
            self.all_methods = {
                'pick': self._pick,
                'place': self._place,
                'done': self._done
            }
            self.locations = self._init_poses()

            mappings = {
                'name': 'panda',
                'prefix': 'panda_',
                'gripper': 'True',
                'collision_arm': 'True',
                'collision_gripper': 'True',
                'safety_limits': 'True',
                'safety_position_margin': '0.15',
                'safety_k_position': '100.0',
                'safety_k_velocity': '40.0',
                'ros2_control': 'True',
                'ros2_control_plugin': 'gz',
                'ros2_control_command_interface': 'position',
                'gazebo_preserve_fixed_joint': 'False',
            }

            panda_config = (
                MoveItConfigsBuilder('panda')
                .robot_description(
                    file_path=os.path.join(
                        get_package_share_directory('panda_description'),
                        'urdf',
                        'multipanda.urdf.xacro'
                    ),
                    mappings=mappings
                )
                .robot_description_semantic(file_path='srdf/multipanda.srdf')
                .trajectory_execution(file_path='config/multimoveit_controller_manager.yaml')
                .planning_pipelines(pipelines=['ompl'])
                .robot_description_kinematics(file_path='config/multikinematics.yaml')
                .joint_limits(file_path='config/multijoint_limits.yaml')
                .moveit_cpp(file_path='config/moveit_py.yaml')
                .to_moveit_configs()
            ).to_dict()

            panda_config.update({'use_sim_time': True})
            file = create_params_file_from_dict(panda_config, '/**')
            self.moveit_component_prefix = 'robot1_'
            self.panda = MoveItPy(node_name='moveit_panda_py', launch_params_filepaths=[file])
            self.moveit_node = Node('moveit_node')

            self.execute_trajectory_client = ActionClient(self.moveit_node,
                                                          ExecuteTrajectory,
                                                          '/execute_trajectory')
            try:
                available = self.execute_trajectory_client.wait_for_server(timeout_sec=5.0)
            except Exception:
                available = False
            # MoveIt action state tracking
            self.moveit_goal_handle = None
            self.moveit_result_future = None
            self.moveit_status = None  # GoalStatus code
            self.moveit_last_task_result = TaskResult.UNKNOWN
            if not available:
                self.moveit_node.get_logger().warning('ExecuteTrajectory action server not available (timeout).')
            self.panda_arm = self.panda.get_planning_component(self.moveit_component_prefix + 'arm')
            self.panda_gripper = self.panda.get_planning_component(self.moveit_component_prefix + 'gripper')
            self.moveit_execution_manager = self.panda.get_trajectory_execution_manager()

    def get_action_methods(self, actions):
        all_methods = {
            'move_forward': self._move_forward,
            'move_backwards': self._move_backwards,
            'move_left': self._move_left,
            'move_right': self._move_right,
            'turn_left': self._turn_left,
            'turn_right': self._turn_right,
            'move_to': self._move_to,
            'pick': self._pick,
            'done': self._done
        }
        return {action: all_methods[action] for action in actions if action in all_methods}

    def set_moveit_component_prefix(self, prefix):
        self.moveit_component_prefix = prefix
        self.panda_arm = self.panda.get_planning_component(
            self.moveit_component_prefix + 'arm'
        )
        self.panda_gripper = self.panda.get_planning_component(
            self.moveit_component_prefix + 'gripper'
        )

    def cancel_actions(self):
        if self.use_nav:
            self.navigator.cancelTask()
        if self.use_moveit:
            self.cancel_moveit_task()

    def cancel_moveit_task(self):
        """Cancel current MoveIt ExecuteTrajectory action if any."""

        if self.moveit_goal_handle and self.moveit_result_future:
            future = self.moveit_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(
                self.moveit_node, future)
        return

    def is_moveit_task_complete(self):
        """Non-blocking check whether current MoveIt task finished."""
        if not self.moveit_result_future:
            return True  # nothing active
        rclpy.spin_until_future_complete(
            self.moveit_node, self.moveit_result_future, timeout_sec=0.1
        )
        if self.moveit_result_future.result():
            self.moveit_status = self.moveit_result_future.result().status
            if self.moveit_status != GoalStatus.STATUS_SUCCEEDED:
                return True
        else:
            return False
        return True

    def get_moveit_result(self):
        """Map stored MoveIt status to TaskResult enum."""
        return self.moveit_last_task_result

    def execute_action(self, action_name, arg):
        method = self.all_methods.get(action_name)
        if not method:
            raise ValueError(f'Unknown action: {action_name}')
        if arg is None:
            try:
                return method()
            except TypeError as e:
                raise TypeError(f"Action '{action_name}' requires an argument") from e
        else:
            try:
                return method(arg)
            except TypeError:
                return method()

    def wait_for_completition(self):
        if self.use_nav:
            while not self.navigator.isTaskComplete():
                time.sleep(0.2)
        if self.use_moveit:
            while not self.is_moveit_task_complete():
                time.sleep(0.2)
            return

    def check_for_result(self):
        if self.use_nav:
            return self.navigator.getResult()
        if self.use_moveit:
            if self.moveit_status == GoalStatus.STATUS_SUCCEEDED:
                return TaskResult.SUCCEEDED
            elif self.status == GoalStatus.STATUS_ABORTED:
                return TaskResult.FAILED
            elif self.status == GoalStatus.STATUS_CANCELED:
                return TaskResult.CANCELED
            else:
                return TaskResult.UNKNOWN

    def _init_poses(self):

        locations = {}
        if self.use_nav:

            fridge_pose = PoseStamped()
            fridge_pose.header.frame_id = 'map'
            fridge_pose.pose.position.x = -1.3
            fridge_pose.pose.position.y = 1.625
            fridge_pose.pose.position.z = 0.0
            fridge_pose.pose.orientation.x = 0.0
            fridge_pose.pose.orientation.y = 0.0
            fridge_pose.pose.orientation.z = 0.0
            fridge_pose.pose.orientation.w = 1.0
            locations['fridge'] = fridge_pose

            shower_pose = PoseStamped()
            shower_pose.header.frame_id = 'map'
            shower_pose.pose.position.x = -10.053603357076645
            shower_pose.pose.position.y = 2.6685024082660678
            shower_pose.pose.position.z = 0.0
            shower_pose.pose.orientation.x = 0.0
            shower_pose.pose.orientation.y = 0.0
            shower_pose.pose.orientation.z = 0.0
            shower_pose.pose.orientation.w = 1.0
            locations['shower'] = shower_pose

            office_desk_pose = PoseStamped()
            office_desk_pose.header.frame_id = 'map'
            office_desk_pose.pose.position.x = 0.30000000000000004
            office_desk_pose.pose.position.y = 7.200071614980698
            office_desk_pose.pose.position.z = 0.0
            office_desk_pose.pose.orientation.x = 0.0
            office_desk_pose.pose.orientation.y = 0.0
            office_desk_pose.pose.orientation.z = 0.0
            office_desk_pose.pose.orientation.w = 1.0
            locations['office_desk'] = office_desk_pose

            bed_pose = PoseStamped()
            bed_pose.header.frame_id = 'map'
            bed_pose.pose.position.x = -7.908
            bed_pose.pose.position.y = 4.501
            bed_pose.pose.position.z = 0.0
            bed_pose.pose.orientation.x = 0.0
            bed_pose.pose.orientation.y = 0.0
            bed_pose.pose.orientation.z = 0.602
            bed_pose.pose.orientation.w = 0.798
            locations['bed'] = bed_pose

            sofa_pose = PoseStamped()
            sofa_pose.header.frame_id = 'map'
            sofa_pose.pose.position.x = -7.75
            sofa_pose.pose.position.y = -4.308771568536758
            sofa_pose.pose.position.z = 0.0
            sofa_pose.pose.orientation.x = 0.0
            sofa_pose.pose.orientation.y = 0.0
            sofa_pose.pose.orientation.z = 0.0
            sofa_pose.pose.orientation.w = 1.0
            locations['sofa'] = sofa_pose

            lamp_pose = PoseStamped()
            lamp_pose.header.frame_id = 'map'
            lamp_pose.pose.position.x = -3.2999999627470973
            lamp_pose.pose.position.y = -3.0197394967079165
            lamp_pose.pose.position.z = 0.0
            lamp_pose.pose.orientation.x = 0.0
            lamp_pose.pose.orientation.y = 0.0
            lamp_pose.pose.orientation.z = 0.0
            lamp_pose.pose.orientation.w = 1.0
            locations['lamp'] = lamp_pose

        if self.use_moveit:

            # cubes_xyz = {
            #     'red': (0.65, 0.00, 0.53),
            #     'blue': (0.65, 0.25, 0.53),
            #     'green': (0.80, -0.25, 0.53),
            #     'yellow': (0.95, 0.00, 0.53),
            #     'purple': (0.65, -0.25, 0.53),
            #     'cyan': (0.95, 0.25, 0.53),
            #     'orange': (0.95, -0.25, 0.53),
            #     'black': (0.80, 0.00, 0.53),
            #     'grey': (0.80, 0.25, 0.53),
            # }
            pick_height = 0.53
            cube_colors = ['red',
                           'blue',
                           'green',
                           'yellow',
                           'purple',
                           'cyan',
                           'orange',
                           'black',
                           'grey',]
            cube_poses = {}
            for color in cube_colors:
                cube_poses[color] = get_gz_pose(entity_name='cube_' + color)
            for color, cube_pose in cube_poses.items():
                pick_pose = PoseStamped()

                pick_pose.header.frame_id = 'world'
                pick_pose.pose.position.x = cube_pose.pose.position.x
                pick_pose.pose.position.y = cube_pose.pose.position.y
                pick_pose.pose.position.z = pick_height
                pick_pose.pose.orientation.x = 0.0
                pick_pose.pose.orientation.y = 1.0
                pick_pose.pose.orientation.z = 0.0
                pick_pose.pose.orientation.w = 0.0
                locations[color] = pick_pose
        return locations

    def _move_forward(self, distance):
        if not self.navigator.isTaskComplete():
            self.navigator.error('Cannot move left while another task is in progress.')
            return
        vel = 0.1
        self.navigator.info(f'Moving forward {distance} meters.')
        return self.navigator.driveOnHeading(float(distance),
                                             speed=vel,
                                             time_allowance=float(distance) // vel)

    def _done(self):
        return True

    def _move_backwards(self, distance):
        if not self.navigator.isTaskComplete():
            self.navigator.error('Cannot move left while another task is in progress.')
            return False
        self.navigator.info(f'Moving backwards {distance} meters.')
        vel = 0.2
        return self.navigator.backup(float(distance),
                                     backup_speed=vel,
                                     time_allowance=10)

    def _move_left(self, distance):
        if not self.navigator.isTaskComplete():
            self.navigator.error('Cannot move left while another task is in progress.')
            return False
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.y = float(distance)
        pose.pose.orientation.w = 1.0
        self.navigator.info(f'Moving left {distance} meters.')
        return self.navigator.goToPose(pose)

    def _move_right(self, distance):
        if not self.navigator.isTaskComplete():
            self.navigator.error('Cannot move left while another task is in progress.')
            return False
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.y = -float(distance)
        pose.pose.orientation.w = 1.0
        self.navigator.info(f'Moving right {distance} meters.')
        return self.navigator.goToPose(pose)

    def _turn_left(self, angle):
        if not self.navigator.isTaskComplete():
            self.navigator.error('Cannot move left while another task is in progress.')
            return False
        return self.navigator.spin(float(angle))

    def _turn_right(self, angle):
        if not self.navigator.isTaskComplete():
            self.navigator.error('Cannot move left while another task is in progress.')
            return False
        return self.navigator.spin(-float(angle))

    def _move_to(self, location_name):
        if not self.navigator.isTaskComplete():
            self.navigator.error('Cannot move to location while another task is in progress.')
            return False
        if location_name not in self.locations:
            self.navigator.error(f'Unknown location: {location_name}')
            return False
        pose = self.locations[location_name]
        self.navigator.info(f'Moving to {location_name}.')
        return self.navigator.goToPose(pose)

    def _pick(self, cube):
        self.panda_arm.set_start_state_to_current_state()
        params = PlanRequestParameters(self.panda, 'moveit_cpp')
        params.max_velocity_scaling_factor = 0.1
        params.planning_pipeline = 'ompl'

        gripper_close_value = 0.01
        close_joints_value = {
            f'{self.moveit_component_prefix}finger_joint1': gripper_close_value,
            f'{self.moveit_component_prefix}finger_joint2': gripper_close_value,
        }

        target_pose = self.locations[cube]

        elevated_pose = PoseStamped()
        elevated_pose.header.frame_id = target_pose.header.frame_id
        elevated_pose.pose.position.x = float(target_pose.pose.position.x)
        elevated_pose.pose.position.y = float(target_pose.pose.position.y)
        elevated_pose.pose.position.z = float(target_pose.pose.position.z) + 0.15
        elevated_pose.pose.orientation.x = target_pose.pose.orientation.x
        elevated_pose.pose.orientation.y = target_pose.pose.orientation.y
        elevated_pose.pose.orientation.z = target_pose.pose.orientation.z
        elevated_pose.pose.orientation.w = target_pose.pose.orientation.w

        self.panda_arm.set_goal_state(pose_stamped_msg=elevated_pose,
                                      pose_link=self.moveit_component_prefix + 'hand_tcp')
        plan_result = self.panda_arm.plan(single_plan_parameters=params)

        if not plan_result:
            return False
        robot_trajectory = plan_result.trajectory

        if not self._execute_moveit_command(robot_trajectory):
            return False

        self.panda_arm.set_start_state_to_current_state()
        self.panda_arm.set_goal_state(pose_stamped_msg=target_pose,
                                      pose_link=self.moveit_component_prefix + 'hand_tcp')

        plan_result = self.panda_arm.plan(single_plan_parameters=params)

        if not plan_result:
            return False

        robot_trajectory = plan_result.trajectory

        if not self._execute_moveit_command(robot_trajectory):
            return False

        self.panda_gripper.set_start_state_to_current_state()
        self.robot_model = self.panda.get_robot_model()

        gripper_close = RobotState(self.robot_model)
        gripper_close.set_joint_group_positions(self.moveit_component_prefix + 'gripper',
                                                list(close_joints_value.values()))
        self.panda_gripper.set_goal_state(robot_state=gripper_close)

        gripper_closing_result = self.panda_gripper.plan()
        if not gripper_closing_result:
            return False

        if not self._execute_moveit_command(gripper_closing_result.trajectory):
            return False

        self.panda_arm.set_start_state_to_current_state()
        self.panda_arm.set_goal_state(configuration_name='ready')
        plan_result = self.panda_arm.plan(single_plan_parameters=params)

        if not plan_result:
            return False
        robot_trajectory = plan_result.trajectory

        if not self._execute_moveit_command(robot_trajectory):
            return False
        
        return True

    def _place(self):
        params = PlanRequestParameters(self.panda, 'moveit_cpp')
        params.max_velocity_scaling_factor = 0.1
        params.planning_pipeline = 'ompl'
        self.panda_arm.set_start_state_to_current_state()
        gripper_open_value = 0.036
        open_joints_value = {
            f'{self.moveit_component_prefix}finger_joint1': gripper_open_value,
            f'{self.moveit_component_prefix}finger_joint2': gripper_open_value,
        }
        pose = PoseStamped()
        if 'robot2' in self.moveit_component_prefix.lower():
            pose.header.frame_id = 'world'
            pose.pose.position.x = float(1.01)
            pose.pose.position.y = float(0.50)
            pose.pose.position.z = float(0.797)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 1.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 0.0
        else:
            pose.header.frame_id = 'world'
            pose.pose.position.x = float(0.55)
            pose.pose.position.y = float(-0.55)
            pose.pose.position.z = float(0.797)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 1.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 0.0

        self.panda_arm.set_goal_state(pose_stamped_msg=pose,
                                      pose_link=self.moveit_component_prefix + 'hand_tcp')
        plan_result = self.panda_arm.plan(single_plan_parameters=params)

        if not plan_result:
            return False
        robot_trajectory = plan_result.trajectory
        if not self._execute_moveit_command(robot_trajectory):
            return False

        self.panda_gripper.set_start_state_to_current_state()
        self.robot_model = self.panda.get_robot_model()

        gripper_open = RobotState(self.robot_model)
        gripper_open.set_joint_group_positions(self.moveit_component_prefix + 'gripper',
                                               list(open_joints_value.values()))
        self.panda_gripper.set_goal_state(robot_state=gripper_open)

        gripper_openning_result = self.panda_gripper.plan()
        if not gripper_openning_result:
            return False
        if not self._execute_moveit_command(gripper_openning_result.trajectory):
            return False

        self.panda_arm.set_start_state_to_current_state()
        self.panda_arm.set_goal_state(configuration_name='ready')
        plan_result = self.panda_arm.plan(single_plan_parameters=params)

        if not plan_result:
            return False

        robot_trajectory = plan_result.trajectory
        if not self._execute_moveit_command(robot_trajectory):
            return False

        return True

    def _execute_moveit_command(self, trajectory):
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory.get_robot_trajectory_msg()

        send_goal_future = self.execute_trajectory_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.moveit_node, send_goal_future)
        self.moveit_goal_handle = send_goal_future.result()
        if not self.moveit_goal_handle or not self.moveit_goal_handle.accepted:
            self.moveit_node.get_logger().warning(
                'ExecuteTrajectory goal was rejected by server.'
            )
            self.moveit_last_task_result = TaskResult.FAILED
            return False
        
        self.moveit_result_future = self.moveit_goal_handle.get_result_async()
        self.wait_for_completition()
        return True

    def check_arm_collision(self):
        robot_model = self.panda.get_robot_model()
        self.panda_arm.set_start_state_to_current_state()
        scene = PlanningScene(robot_model)
        return scene.is_state_colliding(self.panda_arm.get_start_state(), 'robot2_arm')
