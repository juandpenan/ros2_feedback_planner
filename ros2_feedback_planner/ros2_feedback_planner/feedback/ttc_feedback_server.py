"""ROS2 feedback node implementing a laser-scan TTC baseline."""

from feedback_planner_interfaces.srv import TriggerFeedback

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from ros2_feedback_planner.feedback.heuristic_ttc import TTCConfig
from ros2_feedback_planner.feedback.heuristic_ttc import TTCMonitor
from ros2_feedback_planner.feedback.heuristic_ttc import closest_valid_range

from sensor_msgs.msg import LaserScan

from std_msgs.msg import Empty as Emptymsg

from std_srvs.srv import Empty


class TTCFeedbackNode(LifecycleNode):
    """Lifecycle node that cancels navigation actions with laser TTC."""

    def __init__(self, *args, **kwargs):
        """Initialize the TTC feedback node."""
        super().__init__('feedback_node', *args, **kwargs)
        self.is_executing = False
        self.last_feedback_input = ''
        self.monitor = TTCMonitor()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure scan subscriptions and planner-facing services."""
        _ = state
        try:
            self.scan_topic = self.get_parameter(
                'ttc.scan_topic'
            ).get_parameter_value().string_value
            self.forward_start_index = self.get_parameter(
                'ttc.forward_start_index'
            ).get_parameter_value().integer_value
            self.forward_end_index = self.get_parameter(
                'ttc.forward_end_index'
            ).get_parameter_value().integer_value
            self.valid_min_distance_m = self.get_parameter(
                'ttc.valid_min_distance_m'
            ).get_parameter_value().double_value
            self.valid_max_distance_m = self.get_parameter(
                'ttc.valid_max_distance_m'
            ).get_parameter_value().double_value
            safety_distance_m = self.get_parameter(
                'ttc.safety_distance_m'
            ).get_parameter_value().double_value
            horizon_s = self.get_parameter(
                'ttc.horizon_s'
            ).get_parameter_value().double_value
            min_closing_speed_mps = self.get_parameter(
                'ttc.min_closing_speed_mps'
            ).get_parameter_value().double_value
        except (AttributeError, TypeError, ValueError) as exc:
            self.get_logger().error(f'Error reading TTC parameters: {exc}')
            return TransitionCallbackReturn.FAILURE

        self.monitor = TTCMonitor(
            TTCConfig(
                safety_distance_m=safety_distance_m,
                horizon_s=horizon_s,
                min_closing_speed_mps=min_closing_speed_mps,
            )
        )

        cb_group = ReentrantCallbackGroup()
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10,
            callback_group=cb_group,
        )
        self.trigger_srv = self.create_service(
            TriggerFeedback,
            'trigger_feedback',
            self.handle_trigger_feedback,
            callback_group=cb_group,
        )
        self.stop_srv = self.create_service(
            Empty,
            'stop_feedback',
            self.stop_executing,
            callback_group=cb_group,
        )
        self.cancel_action_client = self.create_client(
            TriggerFeedback,
            'cancel_execution',
            callback_group=cb_group,
        )
        self.replan_pub = self.create_publisher(
            Emptymsg,
            'replan_counter',
            10,
            callback_group=cb_group,
        )

        self.get_logger().info('TTC feedback node configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Enable monitoring after lifecycle activation."""
        _ = state
        self.is_executing = False
        self.monitor.reset()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Disable monitoring while inactive."""
        _ = state
        self.is_executing = False
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Destroy ROS entities created during configuration."""
        _ = state
        self.destroy_subscription(self.scan_sub)
        self.destroy_service(self.trigger_srv)
        self.destroy_service(self.stop_srv)
        self.destroy_client(self.cancel_action_client)
        self.destroy_publisher(self.replan_pub)
        self.is_executing = False
        self.last_feedback_input = ''
        self.monitor.reset()
        return TransitionCallbackReturn.SUCCESS

    def handle_trigger_feedback(self, request, response):
        """Start TTC monitoring for the current planner action."""
        self.last_feedback_input = request.feedback_input
        self.monitor.reset()
        self.is_executing = True
        self.get_logger().info(
            f'TTC monitoring started for: {self.last_feedback_input}'
        )
        return response

    def stop_executing(self, request, response):
        """Stop TTC monitoring for the current planner action."""
        _ = request
        self.is_executing = False
        self.monitor.reset()
        return response

    def scan_callback(self, msg: LaserScan):
        """Update TTC estimates from the configured forward scan sector."""
        if not self.is_executing:
            return

        distance_m = closest_valid_range(
            msg.ranges[self.forward_start_index:self.forward_end_index],
            self.valid_min_distance_m,
            self.valid_max_distance_m,
        )
        if distance_m is None:
            return

        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if stamp_s <= 0.0:
            stamp_s = self.get_clock().now().nanoseconds / 1e9

        observation = self.monitor.update(distance_m, stamp_s)
        if not observation.should_trigger:
            return

        feedback = (
            'Heuristic TTC baseline predicted path_clear failure: '
            f'distance={observation.distance_m:.2f}m, '
            f'closing_speed={observation.closing_speed_mps:.2f}m/s, '
            f'ttc={observation.ttc_s:.2f}s.'
        )
        req = TriggerFeedback.Request()
        req.feedback_input = feedback
        self.replan_pub.publish(Emptymsg())
        self.cancel_action_client.call_async(req)
        self.is_executing = False
        self.get_logger().info(feedback)


def main(args=None):
    """Run the TTC feedback node."""
    rclpy.init(args=args)
    node = TTCFeedbackNode(
        automatically_declare_parameters_from_overrides=True
    )
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
