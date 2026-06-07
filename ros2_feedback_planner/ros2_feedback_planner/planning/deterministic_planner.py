"""Lifecycle planner node with no LLM/API dependency."""

import time

import rclpy
from feedback_planner_interfaces.srv import TriggerFeedback
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import String
from std_srvs.srv import Empty

from ros2_feedback_planner.planning.actions import BaseAction
from ros2_feedback_planner.planning.actions import TaskResult
from ros2_feedback_planner.planning.deterministic_plan import feedback_input
from ros2_feedback_planner.planning.deterministic_plan import initial_plan
from ros2_feedback_planner.planning.deterministic_plan import parse_action
from ros2_feedback_planner.planning.deterministic_plan import recovery_plan


class DeterministicPlannerNode(LifecycleNode):
    """Execute move_to(bed), done(), with TTC-triggered backup recovery."""

    def __init__(self, node_name='planner_node', *args, **kwargs):
        """Initialize planner state."""
        super().__init__(node_name=node_name, *args, **kwargs)
        self.cb_group = ReentrantCallbackGroup()
        self.is_activated = False
        self.is_executing = False
        self.success_called = False
        self.cancel_requested = False
        self.last_feedback_result = ''
        self.plan = []
        self.timer = None
        self.action_manager = None

    def _string_param(self, name, default):
        if not self.has_parameter(name):
            return default
        value = self.get_parameter(name).get_parameter_value().string_value
        return value if value else default

    def _double_param(self, name, default):
        if not self.has_parameter(name):
            return default
        value = self.get_parameter(name).get_parameter_value().double_value
        return value if value > 0.0 else default

    def on_configure(self, state: State):
        """Configure actions and planner-facing services."""
        _ = state
        self.get_logger().fatal('Configuring deterministic planner...')
        self.destination = self._string_param('deterministic.destination', 'bed')
        self.backup_distance_m = self._double_param(
            'deterministic.backup_distance_m', 0.5
        )
        action_backend = self._string_param('action_backend', 'nav2')
        self.plan = initial_plan(self.destination)
        self.current_feedback_input = feedback_input(self.destination)
        self.success_called = False
        self.cancel_requested = False

        self.action_publisher = self.create_publisher(
            String,
            'first_action',
            10,
            callback_group=self.cb_group,
        )
        self.cancel_srv = self.create_service(
            TriggerFeedback,
            'cancel_execution',
            self.handle_cancel,
            callback_group=self.cb_group,
        )
        self.trigger_client = self.create_client(
            TriggerFeedback,
            'trigger_feedback',
            callback_group=self.cb_group,
        )
        self.stop_client = self.create_client(
            Empty,
            'stop_feedback',
            callback_group=self.cb_group,
        )
        self.on_success_client = self.create_client(
            Empty,
            'on_success',
            callback_group=self.cb_group,
        )

        self.action_manager = BaseAction(backend=action_backend)
        self.action_manager.on_configure()

        self.timer = self.create_timer(
            1.0,
            self.timer_cb,
            callback_group=self.cb_group,
        )
        self.timer.cancel()

        msg = String()
        msg.data = self.current_feedback_input
        self.action_publisher.publish(msg)
        self.get_logger().fatal(f'Deterministic initial plan: {self.plan}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        """Start deterministic execution."""
        _ = state
        self.get_logger().fatal('Activating deterministic planner...')
        self.is_activated = True
        if self.timer is not None:
            self.timer.reset()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State):
        """Stop execution and cancel any active action."""
        _ = state
        self.get_logger().fatal('Deactivating deterministic planner...')
        self.is_activated = False
        if self.timer is not None:
            self.timer.cancel()
        if self.action_manager is not None:
            self.action_manager.cancel_actions()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State):
        """Destroy ROS entities created during configuration."""
        _ = state
        self.get_logger().fatal('Cleaning up deterministic planner...')
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None
        for attr, destroy in (
            ('action_publisher', self.destroy_publisher),
            ('cancel_srv', self.destroy_service),
            ('trigger_client', self.destroy_client),
            ('stop_client', self.destroy_client),
            ('on_success_client', self.destroy_client),
        ):
            entity = getattr(self, attr, None)
            if entity is not None:
                destroy(entity)
                setattr(self, attr, None)
        self.plan = []
        self.is_activated = False
        self.is_executing = False
        self.cancel_requested = False
        self.last_feedback_result = ''
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State):
        """Shutdown lifecycle hook."""
        _ = state
        self.is_activated = False
        return TransitionCallbackReturn.SUCCESS

    def handle_cancel(self, request, response):
        """Cancel current navigation when TTC predicts collision."""
        if not self.is_activated:
            return response
        self.last_feedback_result = request.feedback_input
        self.cancel_requested = True
        if self.action_manager is not None:
            self.action_manager.cancel_actions()
        self.get_logger().fatal(
            f'TTC requested cancellation: {self.last_feedback_result}'
        )
        return response

    def _call_success_once(self):
        if self.success_called:
            return
        self.success_called = True
        self.on_success_client.call_async(Empty.Request())

    def _trigger_ttc_for_move(self, action):
        if not action.startswith('move_to('):
            return
        req = TriggerFeedback.Request()
        req.feedback_input = self.current_feedback_input
        self.trigger_client.call_async(req)

    def _stop_ttc(self):
        self.stop_client.call_async(Empty.Request())

    def _execute_action(self, action):
        name, arg = parse_action(action)
        self.get_logger().fatal(f'Executing deterministic action: {action}')
        self._trigger_ttc_for_move(action)
        self.action_manager.execute_action(name, arg)
        self.action_manager.wait_for_completition()
        if action.startswith('move_to('):
            self._stop_ttc()
        return self.action_manager.check_for_result()

    def _schedule_recovery(self):
        self.plan = recovery_plan(self.destination, self.backup_distance_m)
        self.cancel_requested = False
        self.get_logger().fatal(f'Deterministic recovery plan: {self.plan}')

    def timer_cb(self):
        """Execute one deterministic action at a time."""
        if not self.is_activated or self.is_executing:
            return
        if not self.plan:
            self.plan = initial_plan(self.destination)
        self.is_executing = True
        self.timer.cancel()

        try:
            action = self.plan.pop(0)
            if action == 'done()':
                self.get_logger().fatal('Deterministic plan complete.')
                self._call_success_once()
                self.is_activated = False
                return

            result = self._execute_action(action)
            self.get_logger().fatal(f'Deterministic result: {result}')

            if result == TaskResult.CANCELED or self.cancel_requested:
                self._schedule_recovery()
            elif result == TaskResult.FAILED:
                self.get_logger().fatal('Action failed; retrying fixed plan.')
                self.plan = initial_plan(self.destination)
            elif result == TaskResult.SUCCEEDED:
                self.cancel_requested = False
        finally:
            self.is_executing = False
            if self.is_activated and self.timer is not None:
                # Give Nav2/feedback services a breath before the next action.
                time.sleep(0.1)
                self.timer.reset()


def main(args=None):
    """Run the deterministic planner node."""
    rclpy.init(args=args)
    node = DeterministicPlannerNode(
        automatically_declare_parameters_from_overrides=True,
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
