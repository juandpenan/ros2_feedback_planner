"""Manipulator simulator node for multi-robot pick-and-place scenarios."""

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from ros2_feedback_planner.planning.actions import BaseAction
import random
from std_msgs.msg import Bool
from rclpy.callback_groups import ReentrantCallbackGroup
from feedback_planner_interfaces.srv import TriggerManipulation
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from ros2_feedback_planner.utils import is_on_table, set_gz_pose, is_in_any_recipient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import DisplayTrajectory
from action_msgs.msg import GoalStatusArray
import sys
import time


class ManipulatorSim(LifecycleNode):
    """Lifecycle node that simulates a second robot performing pick-and-place tasks."""

    def __init__(self):
        """Initialize the ManipulatorSim node."""
        super().__init__('manipulator_sim')
        self._action = None
        self._first_cube = None
        self.is_active = False
        self.is_colliding = False
        self.is_first_time = True
        self.robot1_is_executing = False
        self.robot1_current_goal = None
        self.robot1_current_trajectory_point = None
        self._routine_running = False
        cubes = ['grey',
                 'red',
                 'blue',
                 'green',
                 'yellow',
                 'purple',
                 'cyan',
                 'orange',
                 'black',]

        self.declare_parameter('cubes', cubes)
        for cube in cubes:
            self.declare_parameter(cube, [0.0, 0.0, 0.0])

    def on_configure(self, _):
        if not self._action:
            self._action = BaseAction(backend='moveit')
            self._action.set_moveit_component_prefix('robot2_')
        self._cb_group = ReentrantCallbackGroup()

        self._cubes = self.get_parameter('cubes').value
        random.shuffle(self._cubes)
        for cube in self._cubes:
            pose = self.get_parameter(cube).value
            ps = PoseStamped()
            ps.header.frame_id = 'world'
            ps.pose.position.x = pose[0]
            ps.pose.position.y = pose[1]
            ps.pose.position.z = pose[2]
            set_gz_pose(entity_name='cube_' + cube, target_pose=ps)

        self.trigger_srv = self.create_service(
            TriggerManipulation,
            'set_first_cube',
            self.handle_set_first_cube,
            callback_group=self._cb_group
        )
        self.collision_publisher = self.create_publisher(Bool,
                                                         'is_colliding',
                                                         10,
                                                         callback_group=self._cb_group)
        
        # Subscribe to Robot1's action status
        self.robot1_status_sub = self.create_subscription(
            GoalStatusArray,
            '/robot1_joint_trajectory_controller/follow_joint_trajectory/_action/status',
            self.robot1_status_callback,
            10,
            callback_group=self._cb_group
        )
        # Subscribe to Robot1's planned trajectory

        self.robot1_planned_path_sub = self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.robot1_planned_path_callback,
            10,
            callback_group=self._cb_group
        )
        
        self.timer = self.create_timer(1, self.check_collision, self._cb_group)
        self.execute_timer = self.create_timer(1, self.start_pick_place_routine, self._cb_group)

        if not self._action.on_configure():
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _):
        self.is_active = True
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, _):
        """Deactivate the node."""
        self.get_logger().fatal('Starting deactivation...')
        self.is_active = False
        
        # Wait for routine to complete if running
        max_wait = 5.0  # seconds
        start = time.time()
        while hasattr(self, '_routine_running') and self._routine_running:
            if time.time() - start > max_wait:
                self.get_logger().warn('Timeout waiting for routine to complete')
                break
            time.sleep(0.1)
        
        if self._action:
            try:
                self.get_logger().info('Cancelling ongoing actions...')
                self._action.cancel_actions()
            except Exception as e:
                self.get_logger().error(f'Error cancelling actions: {e}')

        time.sleep(0.5)

        if self._action:
            try:
                self._action.on_deactivate()
            except Exception as e:
                self.get_logger().error(f'Error during pitch retract: {e}')        
        self.get_logger().fatal('Deactivation complete')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _):
        """Clean up resources."""
        self.get_logger().fatal('Cleaning up ManipulatorSim...')
        if hasattr(self, 'trigger_srv') and self.trigger_srv:
            self.destroy_service(self.trigger_srv)
        if hasattr(self, 'collision_publisher') and self.collision_publisher:
            self.destroy_publisher(self.collision_publisher)
        if hasattr(self, 'robot1_status_sub') and self.robot1_status_sub:
            self.destroy_subscription(self.robot1_status_sub)
        if hasattr(self, 'timer') and self.timer:
            self.destroy_timer(self.timer)
            self.timer = None
        if hasattr(self, 'execute_timer') and self.execute_timer:
            self.destroy_timer(self.execute_timer)
            self.execute_timer = None
        self.robot1_current_trajectory_point = None
        self.is_colliding = False
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _):
        """Shutdown the node."""
        self.get_logger().fatal('Shutting down ManipulatorSim...')
        return TransitionCallbackReturn.SUCCESS

    def start_pick_place_routine(self):
        """Execute the pick and place routine for all cubes."""
        if not self.is_active:
            return
        
        # Prevent re-entry if already executing
        if hasattr(self, '_routine_running') and self._routine_running:
            self.get_logger().warn('Routine already running, skipping')
            return
        
        self._routine_running = True
        self.execute_timer.cancel()
        
        for c in self._cubes:
            if not self.is_active:  # Check if still active
                self._routine_running = False
                return
            if not is_on_table(c):
                continue
            retries = 4
            attempt = 0
            while is_on_table(c) and attempt < retries and self.is_active:
                if not self.is_first_time and self.robot1_is_executing:  # not self.is_colliding: activate?
                    self.get_logger().info(
                        f'Robot1 is executing, waiting 3 seconds before picking {c}...'
                    )
                    time.sleep(4.0)
                attempt += 1
                if self.is_first_time:
                    if not self._action.execute_action('pick', c):
                        continue
                    if not self._action.execute_action('place', None):
                        continue
                    self.is_first_time = False
                    continue

                if self.robot1_current_trajectory_point:
                    if not self._action.execute_action('pick_secure', (c, self.robot1_current_trajectory_point)):
                        continue

                    if not self._action.execute_action('place_secure', (c, self.robot1_current_trajectory_point)):
                        continue
                    self.robot1_current_trajectory_point = None
                else:
                    if not self._action.execute_action('pick', c):
                        continue
                    if not self._action.execute_action('place', None):
                        continue

        for cube in self._cubes:
            if (not is_on_table(cube)) and (not is_in_any_recipient(cube)):
                ps = PoseStamped()
                ps.header.frame_id = 'world'
                ps.pose.position.x = 0.0
                ps.pose.position.y = 0.0
                ps.pose.position.z = 0.0
                ps.pose.orientation.w = 1.0
                set_gz_pose(entity_name='cube_' + cube, target_pose=ps)
                self.get_logger().fatal(f'Resetting cube {cube} to origin (0,0,0).')

        cubes_remaining = any(is_on_table(cube) for cube in self._cubes)
        self._routine_running = False

        if not self.is_active:
            return
            
        if cubes_remaining:
            self.get_logger().fatal('Cubes still on table, restarting timer...')
            if self.execute_timer is not None:
                self.execute_timer.reset()
        else:
            self.get_logger().fatal('All cubes processed, destroying timer...')
            if self.execute_timer is not None:
                self.destroy_timer(self.execute_timer)
                self.execute_timer = None

    def check_collision(self):
        if not self.is_active:
            return
        msg = Bool()
        if self._action.check_arm_collision():
            msg.data = True
            self.collision_publisher.publish(msg)
        else:
            msg.data = False
            self.collision_publisher.publish(msg)

    def robot1_status_callback(self, msg):
        """Monitor Robot1's trajectory execution status."""
        if not msg.status_list:
            self.robot1_is_executing = False
            return
        
        # Get the latest goal status
        latest_status = msg.status_list[-1]
        
        # Status codes: 1=ACCEPTED, 2=EXECUTING, 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        if latest_status.status in [1, 2]:  # ACCEPTED or EXECUTING
            self.robot1_is_executing = True
            self.robot1_current_goal = latest_status.goal_info.goal_id
            self.get_logger().info(
                f'Robot1 is executing trajectory: {self.robot1_current_goal.uuid}',
                throttle_duration_sec=2.0
            )
        else:  # SUCCEEDED, CANCELED, or ABORTED
            if self.robot1_is_executing:
                self.get_logger().info(
                    f'Robot1 finished executing (status: {latest_status.status})'
                )
            self.robot1_is_executing = False
            self.robot1_current_goal = None

    def robot1_planned_path_callback(self, msg):
        if msg.trajectory and len(msg.trajectory) > 0:
            trajectory = msg.trajectory[0]
            if (trajectory.joint_trajectory and
                    trajectory.joint_trajectory.joint_names and
                    len(trajectory.joint_trajectory.joint_names) > 0):
                first_joint = trajectory.joint_trajectory.joint_names[0]
                if first_joint.startswith('robot2_'):
                    return

                # Filter out gripper commands (short trajectories) - only care about arm movements
                MIN_TRAJECTORY_POINTS = 10
                if (trajectory.joint_trajectory.points and 
                        len(trajectory.joint_trajectory.points) >= MIN_TRAJECTORY_POINTS):
                    # Store all trajectory points for comprehensive collision checking
                    self.robot1_current_trajectory_point = trajectory.joint_trajectory.points
                    self.get_logger().info(
                        f'Received Robot1 arm trajectory: {len(self.robot1_current_trajectory_point)} points',
                        throttle_duration_sec=2.0
                    )
                else:
                    # Skip short trajectories (likely gripper commands)
                    if trajectory.joint_trajectory.points:
                        self.get_logger().debug(
                            f'Skipping short trajectory ({len(trajectory.joint_trajectory.points)} points) - '
                            f'likely gripper command',
                            throttle_duration_sec=2.0
                        )

    def handle_set_first_cube(self, request, response):
        self.get_logger().fatal(f"Received set_first_cube request: '{request.color}'")
        data = request.color
        start = data.find('(')
        end = data.find(')')
        color = data[start + 1:end].strip()
        random.shuffle(self._cubes)
        self._cubes.remove(color)
        self._cubes.insert(0, color)
        self.get_logger().fatal(f'First cube set to: {self._cubes[0]}')
        self.is_colliding = request.is_colliding
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorSim()
    exec_ = rclpy.executors.MultiThreadedExecutor()
    exec_.add_node(node)
    try:
        exec_.spin()
    finally:
        exec_.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
