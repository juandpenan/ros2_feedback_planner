import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from ros2_feedback_planner.planning.actions import BaseAction
import time
import random
from std_msgs.msg import String, Bool
from rclpy.callback_groups import ReentrantCallbackGroup
from feedback_planner_interfaces.srv import TriggerFeedback


class ManipulatorSim(LifecycleNode):
    def __init__(self):
        super().__init__('manipulator_sim')
        self._action = None
        self._first_cube = None
        self._cubes = ['grey',
                       'red',
                       'blue',
                       'green',
                       'yellow',
                       'purple',
                       'cyan',
                       'orange',
                       'black',
                       ]
        random.shuffle(self._cubes)


    def on_configure(self, _):
        self._action = BaseAction(backend='moveit')
        self._action.set_moveit_component_prefix('robot2_')
        self._cb_group = ReentrantCallbackGroup()

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

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _):
        for c in self._cubes:
            self.get_logger().info(f'STARTING PICK: {c}')
            for tries in range(5):
                pick_ok = self._action.execute_action('pick', c)
                place_ok = self._action.execute_action('place', None)
                if pick_ok and place_ok:
                    break
        self.get_logger().info('Sequence done.')
        return TransitionCallbackReturn.SUCCESS

    def check_collision(self):
        msg = Bool()
        if self._action.check_arm_collision():
            msg.data = True
            self.collision_publisher.publish(msg)
        else:
            msg.data = False
            self.collision_publisher.publish(msg)

    def handle_set_first_cube(self, request, response):
        data = request.feedback_input
        start = data.find('(')
        end = data.find(')')
        color = data[start + 1:end].strip()
        random.shuffle(self._cubes)
        self._cubes.remove(color)
        self._cubes.insert(0, color)
        # self.get_logger().info(f'NEW LIST:{self._cubes}')
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