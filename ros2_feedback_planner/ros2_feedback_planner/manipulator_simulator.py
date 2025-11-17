"""Manipulator simulator node for multi-robot pick-and-place scenarios."""

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from ros2_feedback_planner.planning.actions import BaseAction
import random
from std_msgs.msg import Bool
from rclpy.callback_groups import ReentrantCallbackGroup
from feedback_planner_interfaces.srv import TriggerFeedback
from ros2_feedback_planner.utils import is_on_table, set_gz_pose
from geometry_msgs.msg import PoseStamped
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
            TriggerFeedback,
            'set_first_cube',
            self.handle_set_first_cube,
            callback_group=self._cb_group
        )
        self.collision_publisher = self.create_publisher(Bool,
                                                         'is_colliding',
                                                         10,
                                                         callback_group=self._cb_group)
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
        self.get_logger().info('Starting deactivation...')
        self.is_active = False
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
        self.get_logger().info('Deactivation complete')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _):
        """Clean up resources."""
        self.get_logger().fatal('Cleaning up ManipulatorSim...')
        self.destroy_service(self.trigger_srv)
        self.destroy_publisher(self.collision_publisher)
        self.destroy_timer(self.timer)
        self.destroy_timer(self.execute_timer)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _):
        """Shutdown the node."""
        self.get_logger().fatal('Shutting down ManipulatorSim...')
        return TransitionCallbackReturn.SUCCESS

    def start_pick_place_routine(self):
        """Execute the pick and place routine for all cubes."""
        if not self.is_active:
            return
        self.execute_timer.cancel()
        for c in self._cubes:
            if not is_on_table(c):
                continue
            retries = 4
            attempt = 0
            while is_on_table(c) and attempt < retries and self.is_active:
                attempt += 1
                if not self._action.execute_action('pick', c):
                    continue
                if not self._action.execute_action('place', None):
                    continue

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

    def handle_set_first_cube(self, request, response):
        self.get_logger().fatal(f"Received set_first_cube request: '{request.feedback_input}'")
        data = request.feedback_input
        start = data.find('(')
        end = data.find(')')
        color = data[start + 1:end].strip()
        random.shuffle(self._cubes)
        self._cubes.remove(color)
        self._cubes.insert(0, color)
        self.get_logger().fatal(f'First cube set to: {self._cubes[0]}')
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
