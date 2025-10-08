import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from ros2_feedback_planner.planning.actions import BaseAction
import time
import random

class ManipulatorSim(LifecycleNode):
    def __init__(self):
        super().__init__('manipulator_sim')
        self._action = None
        self._cubes = ['red',
                       'blue',
                       'green',
                       'yellow',
                       'purple',
                       'cyan',
                       'orange',
                       'black',
                       'grey']

    def on_configure(self, _):
        self._action = BaseAction(backend='moveit')
        self._action.set_moveit_component_prefix('robot2_')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _):
        random.shuffle(self._cubes)
        for c in self._cubes:
            for tries in range(5):
                pick_ok = self._action.execute_action('pick', c)
                time.sleep(0.5)
                place_ok = self._action.execute_action('place', None)
                if pick_ok and place_ok:
                    break
        self.get_logger().info('Sequence done.')
        return TransitionCallbackReturn.SUCCESS

def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorSim()
    exec_ = rclpy.executors.SingleThreadedExecutor()
    exec_.add_node(node)
    try:
        exec_.spin()
    finally:
        exec_.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()