"""Actions for controlling robot navigation using nav2_simple_commander."""

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Pose
from moveit_configs_utils import MoveItConfigsBuilder
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.core.robot_state import RobotState
from moveit.core.planning_scene import PlanningScene
from ros2_feedback_planner.utils import get_gz_pose, is_on_table, set_gz_pose
from moveit.utils import create_params_file_from_dict
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import CollisionObject, AllowedCollisionMatrix, AllowedCollisionEntry
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit.core.robot_trajectory import RobotTrajectory
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg
import time
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import ExecuteTrajectory
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory


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
            self.locations = self.init_poses()
            self.navigator = BasicNavigator(node_name='basic_navigator')
            self.navigator.waitUntilNav2Active()

        if self.use_moveit:
            self.all_methods = {
                'pick': self._pick,
                'place': self._place,
                'place_secure': self._place_secure,
                'pick_secure': self._pick_secure,
                'pitch_retract': self._pitch_retract,
                'done': self._done
            }
            self.locations = self.init_poses()

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
            unique_id = str(time.time()).replace('.', '_')
            self.panda = MoveItPy(
                node_name=f'moveit_panda_py_{unique_id}',
                launch_params_filepaths=[file]
            )
            self.moveit_node = Node(f'moveit_node_{unique_id}')

            self.execute_trajectory_client = ActionClient(self.moveit_node,
                                                          ExecuteTrajectory,
                                                          '/execute_trajectory')
            
            self.r1_gripper_trajectory_client = ActionClient(
                self.moveit_node,
                FollowJointTrajectory,
                '/robot1_gripper_trajectory_controller/follow_joint_trajectory'
            )
            self.r1_joint_trajectory_client = ActionClient(
                self.moveit_node,
                FollowJointTrajectory,
                '/robot1_joint_trajectory_controller/follow_joint_trajectory'
            )
            self.r2_gripper_trajectory_client = ActionClient(
                self.moveit_node,
                FollowJointTrajectory,
                '/robot2_gripper_trajectory_controller/follow_joint_trajectory'
            )
            self.r2_joint_trajectory_client = ActionClient(
                self.moveit_node,
                FollowJointTrajectory,
                '/robot2_joint_trajectory_controller/follow_joint_trajectory'
            )

           
            try:
                available = self.r1_gripper_trajectory_client.wait_for_server(timeout_sec=5.0)
                available = self.r1_joint_trajectory_client.wait_for_server(timeout_sec=5.0)
                available = self.r2_gripper_trajectory_client.wait_for_server(timeout_sec=5.0)
                available = self.r2_joint_trajectory_client.wait_for_server(timeout_sec=5.0)
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
            planning_scene_monitor = self.panda.get_planning_scene_monitor()
            with planning_scene_monitor.read_write() as scene:
                self._add_table_collision_object(scene)

    def _add_table_collision_object(self, scene):
        collision_object = CollisionObject()
        collision_object.header.frame_id = 'world'
        collision_object.id = 'table'

        box_pose = Pose()
        box_pose.position.x = 0.78
        box_pose.position.y = 0.0
        box_pose.position.z = 0.5
        box_pose.orientation.z = 0.7071081
        box_pose.orientation.w = 0.7071055

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [1.5, 0.8, 0.03]

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD

        scene.apply_collision_object(collision_object)
        scene.current_state.update()

        collision_object = CollisionObject()
        collision_object.header.frame_id = 'world'
        collision_object.id = 'red_basket'

        box_pose = Pose()
        box_pose.position.x = 1.01
        box_pose.position.y = 0.55
        box_pose.position.z = 0.6

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.29, 0.29, 0.16]

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD

        scene.apply_collision_object(collision_object)
        scene.current_state.update()

        collision_object = CollisionObject()
        collision_object.header.frame_id = 'world'
        collision_object.id = 'black_basket'

        box_pose = Pose()
        box_pose.position.x = 0.55
        box_pose.position.y = -0.5
        box_pose.position.z = 0.59

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.29, 0.2, 0.135]

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD

        scene.apply_collision_object(collision_object)
        scene.current_state.update()

    def allow_collision(self, link_a, link_b=None):
        """Allow collision between two links or between a link and all others.
        
        Args:
            link_a: First link name (e.g., 'robot1_link7')
            link_b: Second link name (e.g., 'robot2_link7'). If None, allows 
                   collision between link_a and all links.
        
        Returns:
            bool: True if ACM was successfully updated, False otherwise
        """
        if not self.use_moveit:
            self.moveit_node.get_logger().warning(
                'allow_collision only works with moveit backend'
            )
            return False
        
        from moveit_msgs.msg import PlanningScene
        
        acm = AllowedCollisionMatrix()
        
        if link_b is None:
            # Allow collision between link_a and all links
            acm.entry_names = [link_a]
            entry = AllowedCollisionEntry()
            entry.enabled = [True]
            acm.entry_values = [entry]
            acm.default_entry_names = [link_a]
            acm.default_entry_values = [True]
        else:
            # Allow collision between specific link pair
            acm.entry_names = [link_a, link_b]
            e0 = AllowedCollisionEntry()
            e0.enabled = [True, True]
            e1 = AllowedCollisionEntry()
            e1.enabled = [True, True]
            acm.entry_values = [e0, e1]
        
        ps = PlanningScene()
        ps.is_diff = True
        ps.allowed_collision_matrix = acm
        
        # Apply via planning scene monitor
        planning_scene_monitor = self.panda.get_planning_scene_monitor()
        with planning_scene_monitor.read_write() as scene:
            # Apply the ACM directly to the scene
            for i, link_i in enumerate(acm.entry_names):
                for j, link_j in enumerate(acm.entry_names):
                    if i < len(acm.entry_values) and j < len(acm.entry_values[i].enabled):
                        if acm.entry_values[i].enabled[j]:
                            scene.allowed_collision_matrix.set_entry(link_i, link_j, True)
        
        self.moveit_node.get_logger().info(
            f'Allowed collision between {link_a} and {link_b or "all links"}'
        )
        return True
    
    def disallow_collision(self, link_a, link_b=None):
        """Disallow collision between two links or between a link and all others.
        
        Args:
            link_a: First link name (e.g., 'robot1_link7')
            link_b: Second link name (e.g., 'robot2_link7'). If None, disallows 
                   collision between link_a and all links.
        
        Returns:
            bool: True if ACM was successfully updated, False otherwise
        """
        if not self.use_moveit:
            self.moveit_node.get_logger().warning(
                'disallow_collision only works with moveit backend'
            )
            return False
        
        from moveit_msgs.msg import PlanningScene
        
        acm = AllowedCollisionMatrix()
        
        if link_b is None:
            # Disallow collision between link_a and all links
            acm.entry_names = [link_a]
            entry = AllowedCollisionEntry()
            entry.enabled = [False]
            acm.entry_values = [entry]
            acm.default_entry_names = [link_a]
            acm.default_entry_values = [False]
        else:
            # Disallow collision between specific link pair
            acm.entry_names = [link_a, link_b]
            e0 = AllowedCollisionEntry()
            e0.enabled = [False, False]
            e1 = AllowedCollisionEntry()
            e1.enabled = [False, False]
            acm.entry_values = [e0, e1]
        
        ps = PlanningScene()
        ps.is_diff = True
        ps.allowed_collision_matrix = acm
        
        # Apply via planning scene monitor
        planning_scene_monitor = self.panda.get_planning_scene_monitor()
        with planning_scene_monitor.read_write() as scene:
            # Apply the ACM directly to the scene
            for i, link_i in enumerate(acm.entry_names):
                for j, link_j in enumerate(acm.entry_names):
                    if i < len(acm.entry_values) and j < len(acm.entry_values[i].enabled):
                        if not acm.entry_values[i].enabled[j]:
                            scene.allowed_collision_matrix.set_entry(link_i, link_j, False)
        
        self.moveit_node.get_logger().info(
            f'Disallowed collision between {link_a} and {link_b or "all links"}'
        )
        return True
    
    def allow_robot_collision(self, robot1_prefix='robot1_', robot2_prefix='robot2_'):
        """Allow collision between all links of two robots.
        
        This is a convenience method that allows collision between all arm links
        of two robots. Use with caution as it disables collision safety.
        
        Args:
            robot1_prefix: Prefix for first robot (default: 'robot1_')
            robot2_prefix: Prefix for second robot (default: 'robot2_')
        
        Returns:
            bool: True if ACM was successfully updated, False otherwise
        """
        if not self.use_moveit:
            self.moveit_node.get_logger().warning(
                'allow_robot_collision only works with moveit backend'
            )
            return False
        
        # Get robot model to find all links
        robot_model = self.panda.get_robot_model()
        
        # Get all link names for both robots
        r1_links = []
        r2_links = []
        
        for link_name in robot_model.link_model_names:
            if link_name.startswith(robot1_prefix):
                r1_links.append(link_name)
            elif link_name.startswith(robot2_prefix):
                r2_links.append(link_name)
        
        if not r1_links or not r2_links:
            self.moveit_node.get_logger().error(
                f'Could not find links for {robot1_prefix} or {robot2_prefix}'
            )
            return False
        
        self.moveit_node.get_logger().info(
            f'Allowing collision between {len(r1_links)} links of {robot1_prefix} '
            f'and {len(r2_links)} links of {robot2_prefix}'
        )
        
        from moveit_msgs.msg import PlanningScene
        
        # Build ACM with all robot1 and robot2 links
        acm = AllowedCollisionMatrix()
        acm.entry_names = r1_links + r2_links
        
        # Create matrix: allow collisions only between robot1 and robot2 links
        total_links = len(r1_links) + len(r2_links)
        for i, link_i in enumerate(acm.entry_names):
            entry = AllowedCollisionEntry()
            entry.enabled = [False] * total_links
            
            # If link_i is from robot1, allow collision with all robot2 links
            if link_i in r1_links:
                for j, link_j in enumerate(acm.entry_names):
                    if link_j in r2_links:
                        entry.enabled[j] = True
            # If link_i is from robot2, allow collision with all robot1 links
            elif link_i in r2_links:
                for j, link_j in enumerate(acm.entry_names):
                    if link_j in r1_links:
                        entry.enabled[j] = True
            
            acm.entry_values.append(entry)
        
        ps = PlanningScene()
        ps.is_diff = True
        ps.allowed_collision_matrix = acm
        
        # Apply via planning scene monitor
        planning_scene_monitor = self.panda.get_planning_scene_monitor()
        with planning_scene_monitor.read_write() as scene:
            # Apply the ACM directly to the scene
            for i, link_i in enumerate(acm.entry_names):
                for j, link_j in enumerate(acm.entry_names):
                    if i < len(acm.entry_values) and j < len(acm.entry_values[i].enabled):
                        if acm.entry_values[i].enabled[j]:
                            scene.allowed_collision_matrix.set_entry(link_i, link_j, True)
        
        self.moveit_node.get_logger().info(
            f'Allowed collision between {robot1_prefix} and {robot2_prefix}'
        )
        return True

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
            'place': self._place,
            'pitch_retract': self._pitch_retract,
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

    def set_result(self, result):
        """Set the result of the current action execution.
        
        Args:
            result: TaskResult enum value (SUCCEEDED, FAILED, CANCELED, etc.)
        """
        if self.use_moveit:
            self.moveit_last_task_result = result
            # Map TaskResult to GoalStatus for consistency
            if result == TaskResult.SUCCEEDED:
                self.moveit_status = GoalStatus.STATUS_SUCCEEDED
            elif result == TaskResult.FAILED:
                self.moveit_status = GoalStatus.STATUS_ABORTED
            elif result == TaskResult.CANCELED:
                self.moveit_status = GoalStatus.STATUS_CANCELED
            else:
                self.moveit_status = GoalStatus.STATUS_UNKNOWN
        if self.use_nav:

            if result == TaskResult.SUCCEEDED:
                self.navigator.status = GoalStatus.STATUS_SUCCEEDED
            elif result == TaskResult.FAILED:
                self.navigator.status = GoalStatus.STATUS_ABORTED
            elif result == TaskResult.CANCELED:
                self.navigator.status = GoalStatus.STATUS_CANCELED
            else:
                self.navigator.status = GoalStatus.STATUS_UNKNOWN

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
            elif self.moveit_status == GoalStatus.STATUS_ABORTED:
                return TaskResult.FAILED
            elif self.moveit_status == GoalStatus.STATUS_CANCELED:
                return TaskResult.CANCELED
            else:
                return TaskResult.UNKNOWN

    def init_poses(self):

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

            robot_default_pose = PoseStamped()
            robot_default_pose.header.frame_id = 'map'
            robot_default_pose.pose.position.x = -5.5
            robot_default_pose.pose.position.y = -3.8
            robot_default_pose.pose.position.z = 0.0
            robot_default_pose.pose.orientation.x = 0.0
            robot_default_pose.pose.orientation.y = 0.0
            robot_default_pose.pose.orientation.z = 0.7545709 
            robot_default_pose.pose.orientation.w = 0.6562185
            locations['robot_default'] = robot_default_pose

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
                if is_on_table(color):
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
                                             time_allowance=int(float(distance) / vel))

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
        
        if not is_on_table(cube):
            self.set_result(TaskResult.FAILED)
            self.moveit_result_future = None
            return False
        new_locations = self.init_poses()
        
        for key, new_pose in new_locations.items():
            if self.locations[key] != new_pose:
                self.locations[key] = new_pose

        self.panda_arm.set_start_state_to_current_state()
        params = PlanRequestParameters(self.panda, 'moveit_cpp')
        params.max_velocity_scaling_factor = 0.08
        params.planning_pipeline = 'ompl'

        gripper_close_value = 0.01
        close_joints_value = {
            f'{self.moveit_component_prefix}finger_joint1': gripper_close_value,
            f'{self.moveit_component_prefix}finger_joint2': gripper_close_value,
        }
        gripper_open_value = 0.036
        open_joints_value = {
            f'{self.moveit_component_prefix}finger_joint1': gripper_open_value,
            f'{self.moveit_component_prefix}finger_joint2': gripper_open_value,
        }
        self.robot_model = self.panda.get_robot_model()

        self.panda_gripper.set_start_state_to_current_state()
        gripper_open = RobotState(self.robot_model)
        gripper_open.set_joint_group_positions(self.moveit_component_prefix + 'gripper',
                                               list(open_joints_value.values()))
        self.panda_gripper.set_goal_state(robot_state=gripper_open)

        gripper_opening_result = self.panda_gripper.plan()
        if not gripper_opening_result:
            self.moveit_node.get_logger().fatal('Could not plan open gripper')
            self.moveit_result_future = None
            self.set_result(TaskResult.FAILED)
            return False

        if not self._execute_moveit_command(gripper_opening_result.trajectory):
            self.moveit_node.get_logger().fatal('Could not execute open gripper')
            return False
        # self.wait_for_completition()
        # status = self.check_for_result()
        # self.moveit_node.get_logger().info(F'Current status {status}')
        # if status != TaskResult.SUCCEEDED:
        #     return False
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
            self.moveit_node.get_logger().fatal('Could not plan approach cube')
            self.set_result(TaskResult.FAILED)
            self.moveit_result_future = None
            return False
        robot_trajectory = plan_result.trajectory

        if not self._execute_moveit_command(robot_trajectory):
            self.moveit_node.get_logger().fatal('Could not execute approach cube')
            return False
        # self.wait_for_completition()
        # status = self.check_for_result()
        # self.moveit_node.get_logger().info(F'Current status {status}')
        # if status != TaskResult.SUCCEEDED:
        #     return False
        self.panda_arm.set_start_state_to_current_state()
        self.panda_arm.set_goal_state(pose_stamped_msg=target_pose,
                                      pose_link=self.moveit_component_prefix + 'hand_tcp')

        plan_result = self.panda_arm.plan(single_plan_parameters=params)

        if not plan_result:
            self.moveit_node.get_logger().fatal('Could not plan final approach cube')
            self.set_result(TaskResult.FAILED)
            self.moveit_result_future = None
            return False

        robot_trajectory = plan_result.trajectory

        if not self._execute_moveit_command(robot_trajectory):
            self.moveit_node.get_logger().fatal('Could not execute final approach cube')
            return False
        # self.wait_for_completition()
        # status = self.check_for_result()
        # self.moveit_node.get_logger().info(F'Current status {status}')
        # if status != TaskResult.SUCCEEDED:
        #     return False

        self.panda_gripper.set_start_state_to_current_state()
        self.robot_model = self.panda.get_robot_model()

        gripper_close = RobotState(self.robot_model)
        gripper_close.set_joint_group_positions(self.moveit_component_prefix + 'gripper',
                                                list(close_joints_value.values()))
        self.panda_gripper.set_goal_state(robot_state=gripper_close)

        gripper_closing_result = self.panda_gripper.plan()
        if not gripper_closing_result:
            self.moveit_node.get_logger().fatal('Could not plan close gripper')
            self.set_result(TaskResult.FAILED)
            self.moveit_result_future = None
            return False

        if not self._execute_moveit_command(gripper_closing_result.trajectory):
            self.moveit_node.get_logger().fatal('Could not execute close gripper')
            return False
        # self.wait_for_completition()
        # status = self.check_for_result()
        # self.moveit_node.get_logger().info(F'Current status {status}')
        # if status != TaskResult.SUCCEEDED:
        #     return False

        self.panda_arm.set_start_state_to_current_state()
        self.panda_arm.set_goal_state(configuration_name='ready')
        plan_result = self.panda_arm.plan(single_plan_parameters=params)

        if not plan_result:
            self.moveit_node.get_logger().fatal('Could not plan ready pose')
            self.set_result(TaskResult.FAILED)
            self.moveit_result_future = None
            return False
        robot_trajectory = plan_result.trajectory

        if not self._execute_moveit_command(robot_trajectory):
            self.moveit_node.get_logger().fatal('Could not execute ready pose')
            return False
        # self.wait_for_completition()
        # status = self.check_for_result()
        # self.moveit_node.get_logger().info(F'Current status {status}')
        # if status != TaskResult.SUCCEEDED:
        #     return False
        return True

    def _pick_secure(self, touple):
        """Secure pick: checks all waypoint pairs for collision (O(n*m) complexity).

        Args:
            touple: tuple of (cube, other_points) where:
                cube: color/name of the cube (e.g., 'red')
                other_points: list of JointTrajectoryPoint from another robot

        Returns:
            bool: False if any collision is predicted, else proceeds like _pick
        """
        # Unpack the tuple
        cube, other_points = touple
        
        self.moveit_node.get_logger().info(f'PICK_SECURE: Starting for cube={cube}')

        # If cube not on table, behave like _pick
        if not is_on_table(cube):
            self.set_result(TaskResult.FAILED)
            self.moveit_result_future = None
            return False

        # Refresh locations
        new_locations = self.init_poses()
        for key, new_pose in new_locations.items():
            if self.locations.get(key) != new_pose:
                self.locations[key] = new_pose

        # Prepare planning params
        self.panda_arm.set_start_state_to_current_state()
        params = PlanRequestParameters(self.panda, 'moveit_cpp')
        params.max_velocity_scaling_factor = 0.08
        params.planning_pipeline = 'ompl'

        # First movement: approach elevated above target
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
            self.moveit_node.get_logger().error('Could not plan approach cube (secure)')
            self.set_result(TaskResult.FAILED)
            self.moveit_result_future = None
            return False
        
        # Check all trajectory waypoint pairs for collision (O(n*m))
        try:
            planned_msg = plan_result.trajectory.get_robot_trajectory_msg().joint_trajectory
            
            # Handle both list of points and single point
            if isinstance(other_points, list):
                other_waypoints = other_points
            else:
                # Single point - wrap in list
                other_waypoints = [other_points] if other_points else []
            
            if not planned_msg.points or not other_waypoints:
                self.moveit_node.get_logger().warn(
                    'Skipping collision check - no waypoints'
                )
                return self._pick(cube)

            robot_model = self.panda.get_robot_model()
            this_group = self.moveit_component_prefix + 'arm'
            other_group = (
                'robot1_arm' if 'robot2_' in self.moveit_component_prefix 
                else 'robot2_arm'
            )
            
            n_this = len(planned_msg.points)
            n_other = len(other_waypoints)
            
            self.moveit_node.get_logger().info(
                f'Checking {n_this} x {n_other} = {n_this * n_other} '
                f'waypoint pairs for collision'
            )
            
            for i, waypoint_this in enumerate(planned_msg.points):
                if not waypoint_this.positions:
                    continue
                
                for j, waypoint_other in enumerate(other_waypoints):
                    # Extract positions from the other waypoint
                    other_positions = list(
                        getattr(waypoint_other, 'positions', []) or []
                    )
                    if not other_positions:
                        continue
                    
                    combined_state = RobotState(robot_model)
                    
                    # Set this robot's configuration
                    try:
                        combined_state.set_joint_group_positions(
                            this_group, list(waypoint_this.positions)
                        )
                    except Exception as e:
                        self.moveit_node.get_logger().warn(
                            f'Failed to set {this_group} at [{i}]: {e}'
                        )
                        continue
                    
                    # Set other robot's configuration
                    try:
                        combined_state.set_joint_group_positions(
                            other_group, other_positions
                        )
                    except Exception as e:
                        self.moveit_node.get_logger().warn(
                            f'Failed to set {other_group} at [{j}]: {e}'
                        )
                        continue

                    # Check collision for this pair
                    scene = PlanningScene(robot_model)
                    combined_state.update()

                    if scene.is_state_colliding(combined_state, other_group):
                        self.moveit_node.get_logger().fatal(
                            f'COLLISION at waypoint pair [{i},{j}]! '
                            f'Aborting pick_secure.'
                        )
                        return False

            self.moveit_node.get_logger().fatal(
                f'All {n_this * n_other} waypoint pairs collision-free! '
                f'Proceeding with pick.'
            )

        except Exception as e:
            self.moveit_node.get_logger().error(f'Collision check exception: {e}')
            import traceback
            self.moveit_node.get_logger().error(
                f'Traceback:\n{traceback.format_exc()}'
            )

        return self._pick(cube)

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
            pose.pose.position.y = float(0.55)
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
            self.set_result(TaskResult.FAILED)
            return False
        robot_trajectory = plan_result.trajectory
        if not self._execute_moveit_command(robot_trajectory):
            return False
        # self.wait_for_completition()
        # status = self.check_for_result()
        # self.moveit_node.get_logger().info(F'Current status {status}')
        # if status != TaskResult.SUCCEEDED:
        #     return False

        self.panda_gripper.set_start_state_to_current_state()
        self.robot_model = self.panda.get_robot_model()

        gripper_open = RobotState(self.robot_model)
        gripper_open.set_joint_group_positions(self.moveit_component_prefix + 'gripper',
                                               list(open_joints_value.values()))
        self.panda_gripper.set_goal_state(robot_state=gripper_open)

        gripper_openning_result = self.panda_gripper.plan()
        if not gripper_openning_result:
            self.set_result(TaskResult.FAILED)
            return False
        if not self._execute_moveit_command(gripper_openning_result.trajectory):
            return False
        # self.wait_for_completition()
        # status = self.check_for_result()
        # self.moveit_node.get_logger().info(F'Current status {status}')
        # if status != TaskResult.SUCCEEDED:
        #     return False

        self.panda_arm.set_start_state_to_current_state()
        self.panda_arm.set_goal_state(configuration_name='ready')
        plan_result = self.panda_arm.plan(single_plan_parameters=params)

        if not plan_result:
            self.set_result(TaskResult.FAILED)
            return False

        robot_trajectory = plan_result.trajectory
        if not self._execute_moveit_command(robot_trajectory):
            return False
    
        return True
    

    def _place_secure(self, touple):
        """Secure place: checks all waypoint pairs for collision (O(n*m) complexity).

        Args:
            touple: tuple containing (cube, other_points) where:
                cube: color/name of the cube
                other_points: list of JointTrajectoryPoint from the other robot

        Returns:
            bool: False if any collision is predicted, else proceeds like _place
        """
        # Unpack the tuple
        if isinstance(touple, tuple):
            cube, other_points = touple
        else:
            other_points = touple
            cube = None

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
            pose.pose.position.y = float(0.55)
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

        # 1. Plan to place pose and check collision with all waypoint pairs
        self.panda_arm.set_goal_state(
            pose_stamped_msg=pose,
            pose_link=self.moveit_component_prefix + 'hand_tcp'
        )
        plan_result = self.panda_arm.plan(single_plan_parameters=params)
        if not plan_result:
            self.set_result(TaskResult.FAILED)
            return False

        # Check collision for place trajectory
        try:
            planned_msg = plan_result.trajectory.get_robot_trajectory_msg().joint_trajectory
            
            # Handle both list of points and single point
            if isinstance(other_points, list):
                other_waypoints = other_points
            else:
                other_waypoints = [other_points] if other_points else []

            if planned_msg.points and other_waypoints:
                robot_model = self.panda.get_robot_model()
                this_group = self.moveit_component_prefix + 'arm'
                other_group = (
                    'robot1_arm' if 'robot2_' in self.moveit_component_prefix
                    else 'robot2_arm'
                )
                
                n_this = len(planned_msg.points)
                n_other = len(other_waypoints)
                
                self.moveit_node.get_logger().info(
                    f'PLACE: Checking {n_this} x {n_other} = {n_this * n_other} '
                    f'waypoint pairs for collision'
                )
                
                # O(n*m) collision check
                for i, waypoint_this in enumerate(planned_msg.points):
                    if not waypoint_this.positions:
                        continue
                    
                    for j, waypoint_other in enumerate(other_waypoints):
                        other_positions = list(
                            getattr(waypoint_other, 'positions', []) or []
                        )
                        if not other_positions:
                            continue
                        
                        combined_state = RobotState(robot_model)
                        
                        try:
                            combined_state.set_joint_group_positions(
                                this_group, list(waypoint_this.positions)
                            )
                        except Exception as e:
                            self.moveit_node.get_logger().warn(
                                f'Failed to set {this_group} at [{i}]: {e}'
                            )
                            continue
                        
                        try:
                            combined_state.set_joint_group_positions(
                                other_group, other_positions
                            )
                        except Exception as e:
                            self.moveit_node.get_logger().warn(
                                f'Failed to set {other_group} at [{j}]: {e}'
                            )
                            continue
                        
                        scene = PlanningScene(robot_model)
                        combined_state.update()
                        
                        if scene.is_state_colliding(combined_state, other_group):
                            self.moveit_node.get_logger().fatal(
                                f'COLLISION at place waypoint pair [{i},{j}]! '
                                f'Aborting place_secure.'
                            )
                            return False
                
                self.moveit_node.get_logger().info(
                    f'Place trajectory: All {n_this * n_other} pairs collision-free'
                )
        except Exception as e:
            self.moveit_node.get_logger().warn(
                f'Collision check error in place_secure (place pose): {e}'
            )

        # If safe, execute place pose
        robot_trajectory = plan_result.trajectory
        if not self._execute_moveit_command(robot_trajectory):
            return False

        # Open gripper
        self.panda_gripper.set_start_state_to_current_state()
        self.robot_model = self.panda.get_robot_model()
        gripper_open = RobotState(self.robot_model)
        gripper_open.set_joint_group_positions(
            self.moveit_component_prefix + 'gripper',
            list(open_joints_value.values())
        )
        self.panda_gripper.set_goal_state(robot_state=gripper_open)
        gripper_openning_result = self.panda_gripper.plan()
        if not gripper_openning_result:
            self.set_result(TaskResult.FAILED)
            return False
        if not self._execute_moveit_command(gripper_openning_result.trajectory):
            return False

        # 2. Plan to ready pose and check collision with all waypoint pairs
        self.panda_arm.set_start_state_to_current_state()
        self.panda_arm.set_goal_state(configuration_name='ready')
        plan_result_ready = self.panda_arm.plan(single_plan_parameters=params)
        if not plan_result_ready:
            self.set_result(TaskResult.FAILED)
            return False

        # Check collision for ready trajectory
        try:
            planned_msg_ready = (
                plan_result_ready.trajectory.get_robot_trajectory_msg().joint_trajectory
            )
            
            if planned_msg_ready.points and other_waypoints:
                robot_model = self.panda.get_robot_model()
                this_group = self.moveit_component_prefix + 'arm'
                other_group = (
                    'robot1_arm' if 'robot2_' in self.moveit_component_prefix
                    else 'robot2_arm'
                )
                
                n_this = len(planned_msg_ready.points)
                n_other = len(other_waypoints)
                
                self.moveit_node.get_logger().info(
                    f'READY: Checking {n_this} x {n_other} = {n_this * n_other} '
                    f'waypoint pairs for collision'
                )
                
                # O(n*m) collision check
                for i, waypoint_this in enumerate(planned_msg_ready.points):
                    if not waypoint_this.positions:
                        continue
                    
                    for j, waypoint_other in enumerate(other_waypoints):
                        other_positions = list(
                            getattr(waypoint_other, 'positions', []) or []
                        )
                        if not other_positions:
                            continue
                        
                        combined_state = RobotState(robot_model)
                        
                        try:
                            combined_state.set_joint_group_positions(
                                this_group, list(waypoint_this.positions)
                            )
                        except Exception as e:
                            self.moveit_node.get_logger().warn(
                                f'Failed to set {this_group} at [{i}]: {e}'
                            )
                            continue
                        
                        try:
                            combined_state.set_joint_group_positions(
                                other_group, other_positions
                            )
                        except Exception as e:
                            self.moveit_node.get_logger().warn(
                                f'Failed to set {other_group} at [{j}]: {e}'
                            )
                            continue
                        
                        scene = PlanningScene(robot_model)
                        combined_state.update()
                        
                        if scene.is_state_colliding(combined_state, other_group):
                            self.moveit_node.get_logger().fatal(
                                f'COLLISION at ready waypoint pair [{i},{j}]! '
                                f'Aborting place_secure.'
                            )
                            return False
                
                self.moveit_node.get_logger().info(
                    f'Ready trajectory: All {n_this * n_other} pairs collision-free'
                )
        except Exception as e:
            self.moveit_node.get_logger().warn(
                f'Collision check error in place_secure (ready pose): {e}'
            )

        # If safe, execute ready pose
        robot_trajectory_ready = plan_result_ready.trajectory
        if not self._execute_moveit_command(robot_trajectory_ready):
            return False

        return True

    def _pitch_retract(self, angle):
        """Rotate the arm's joint_2 by the specified angle relative to current position.

        Args:
            angle: The angle to rotate joint_2 by (in radians)

        Returns:
            bool: True if execution succeeded, False otherwise
        """

        self.panda_arm.set_start_state_to_current_state()
        current_state = self.panda_arm.get_start_state()

        joint_group_name = self.moveit_component_prefix + 'arm'
        current_joint_values = list(current_state.get_joint_group_positions(joint_group_name))
        target_joint_values = current_joint_values.copy()
        raw_new_val = current_joint_values[1] - float(angle)
        new_val = max(-1.59, min(1.59, raw_new_val))
        target_joint_values[1] = new_val

        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = [
            f'{self.moveit_component_prefix}joint1',
            f'{self.moveit_component_prefix}joint2',
            f'{self.moveit_component_prefix}joint3',
            f'{self.moveit_component_prefix}joint4',
            f'{self.moveit_component_prefix}joint5',
            f'{self.moveit_component_prefix}joint6',
            f'{self.moveit_component_prefix}joint7'
        ]
        num_points = 10  # Number of interpolation points
        duration_total = 2.0  # Total duration in seconds

        for i in range(num_points + 1):
            point = JointTrajectoryPoint()
            t = i / num_points
            point_positions = [
                current_joint_values[j] + t * (target_joint_values[j] - current_joint_values[j])
                for j in range(7)
            ]
            point.positions = point_positions

            if i == 0 or i == num_points:
                point.velocities = [0.0] * 7
            else:
                point.velocities = [
                    (target_joint_values[j] - current_joint_values[j]) / duration_total
                    for j in range(7)
                ]
            total_t = t * duration_total
            time_sec = int(total_t)
            time_nsec = int((total_t - time_sec) * 1e9)
            point.time_from_start = Duration(sec=time_sec, nanosec=time_nsec)

            joint_trajectory.points.append(point)

        robot_trajectory_msg = RobotTrajectoryMsg()
        robot_trajectory_msg.joint_trajectory = joint_trajectory

        robot_model = self.panda.get_robot_model()
        robot_trajectory = RobotTrajectory(robot_model)
        robot_trajectory.joint_model_group_name = joint_group_name

        try:
            robot_trajectory.set_robot_trajectory_msg(current_state, robot_trajectory_msg)
        except Exception as e:
            self.moveit_node.get_logger().error(f'Failed to set robot trajectory message: {e}')
            return False
        
        self.moveit_node.get_logger().info('Sending the retreat to execute!! ')

        if not self._execute_moveit_command(robot_trajectory):
            self.moveit_node.get_logger().error('Failed to execute pitch retract trajectory')
            return False
        # self.wait_for_completition()
        # status = self.check_for_result()
        # self.moveit_node.get_logger().info(F'Current status {status}')
        # if status != TaskResult.SUCCEEDED:
        #     return False
        return True

    def _execute_moveit_command(self, trajectory):
        """Execute a MoveIt trajectory using the appropriate action client.
        
        Args:
            trajectory: The robot trajectory to execute
            
        Returns:
            bool: True if execution succeeded, False otherwise
        """
        # Check if trajectory is for gripper or arm based on joint names
        trajectory = trajectory.get_robot_trajectory_msg()
        is_gripper = any(
            'finger' in joint_name
            for joint_name in trajectory.joint_trajectory.joint_names
        )
        
        # Select the appropriate client based on prefix and part
        if 'robot1_' in self.moveit_component_prefix:
            if is_gripper:
                client = self.r1_gripper_trajectory_client
            else:
                client = self.r1_joint_trajectory_client
        elif 'robot2_' in self.moveit_component_prefix:
            if is_gripper:
                client = self.r2_gripper_trajectory_client
            else:
                client = self.r2_joint_trajectory_client
        else:
            error_msg = (
                f'Unknown robot prefix: {self.moveit_component_prefix}'
            )
            self.moveit_node.get_logger().error(error_msg)
            return False

        if not client.wait_for_server(timeout_sec=5.0):
            self.moveit_node.get_logger().error('Action server not available')
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory.joint_trajectory

        send_goal_future = client.send_goal_async(goal_msg)
        
        rclpy.spin_until_future_complete(
            self.moveit_node, send_goal_future, timeout_sec=10.0
        )
        
        if not send_goal_future.done():
            self.moveit_node.get_logger().error('Send goal timeout')
            return False

        self.moveit_goal_handle = send_goal_future.result()

        if not self.moveit_goal_handle or not self.moveit_goal_handle.accepted:
            self.moveit_node.get_logger().error('Goal rejected by action server')
            return False

        self.moveit_result_future = self.moveit_goal_handle.get_result_async()

        max_wait_time = 60.0
        start_time = time.time()
        if not self.moveit_result_future:
            self.moveit_node.get_logger().fatal('moveit_result_future IS NONE')
            return False

        while not self.moveit_result_future.done():
            rclpy.spin_until_future_complete(
                self.moveit_node, self.moveit_result_future, timeout_sec=0.5
            )

            if time.time() - start_time > max_wait_time:
                self.moveit_node.get_logger().error('Trajectory execution timeout')
                if self.moveit_goal_handle:
                    try:
                        self.moveit_goal_handle.cancel_goal_async()
                    except BaseException:
                        pass
                return False

        result = self.moveit_result_future.result()

        if result is None:
            self.moveit_node.get_logger().error('Trajectory execution result is None')
            self.moveit_status = GoalStatus.STATUS_ABORTED
            self.moveit_last_task_result = TaskResult.FAILED
            self.moveit_node.get_logger().error('returning false!')
            return False

        self.moveit_status = result.status

        if self.moveit_status == GoalStatus.STATUS_SUCCEEDED:
            self.moveit_last_task_result = TaskResult.SUCCEEDED
            return True
        else:
            error_msg = (
                f'Trajectory execution failed with status: '
                f'{self.moveit_status}'
            )
            self.moveit_node.get_logger().error(error_msg)
            self.moveit_last_task_result = TaskResult.FAILED
            return False

    def check_arm_collision(self):
        robot_model = self.panda.get_robot_model()
        self.panda_arm.set_start_state_to_current_state()
        scene = PlanningScene(robot_model)
        return scene.is_state_colliding(self.panda_arm.get_start_state(), 'robot2_arm')

    def _on_configure_moveit(self):
        self.panda_arm.set_start_state_to_current_state()
        self.panda_arm.set_goal_state(configuration_name='ready')
        plan_result = self.panda_arm.plan()

        if not plan_result:
            return False

        robot_trajectory = plan_result.trajectory

        if not self._execute_moveit_command(robot_trajectory):
            return False
        # self.wait_for_completition()
        # status = self.check_for_result()
        # self.moveit_node.get_logger().info(F'Current status {status}')
        # if status != TaskResult.SUCCEEDED:
        #     return False
        return True

    def _on_deactivate_moveit(self):
        self.execute_action('pitch_retract', 0.78)

    def _on_configure_nvagitaion(self):
        ret = set_gz_pose('tiago', self.locations['robot_default'], 'plasys_house')
        self.navigator.setInitialPose(self.locations['robot_default'])
        return ret

    def on_configure(self):
        if self.use_nav:
            return self._on_configure_nvagitaion()
        elif self.use_moveit:
            return self._on_configure_moveit()

    def on_deactivate(self):
        if self.use_nav:
            # doesnt need i think
            return True
        elif self.use_moveit:
            return self._on_deactivate_moveit()
