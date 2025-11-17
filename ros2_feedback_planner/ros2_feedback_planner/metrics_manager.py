import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import LaserScan
from rclpy.duration import Duration
from std_msgs.msg import String, Bool
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from std_srvs.srv import Empty
from std_msgs.msg import Empty as Emptymsg
from feedback_planner_interfaces.srv import TriggerFeedback
import pandas as pd
from tf2_ros import Buffer, TransformListener
import random
import math
import os
import time


class MetricsManager(LifecycleNode):
    """Lifecycle node for managing metrics subscriptions and callbacks."""

    def __init__(self, *args, **kwargs):
        """Initialize the MetricsManager node and its resources."""
        super().__init__('metrics_manager_node', *args, **kwargs)
        self.callback_group = ReentrantCallbackGroup()
        self.first_action = None
        self.will_collide = False
        self.is_planner_configured = False
        self.replan_count = 0
        self.now = self.get_clock().now()
        self.collision_check = None

        self.current_experiment = {
            'method': None,
            'duration_time': None,
            'p_collision': None,
            'success': None,
            'will_fail': None,
            'replan_count': None
        }

        # List to accumulate all experiments
        self.experiments = []

        self.start_test_time = None
        self.is_checking_for_collision = False
        self.success_callback_triggered = False
        self.is_feedback_cleaned_up = False

    def __del__(self):
        """Destructor to ensure metrics are saved even if shutdown is not called properly."""
        try:
            self.save_all_metrics_to_csv()
        except Exception as e:
            # Use print since logger might not be available during destruction
            print(f'Error saving metrics in destructor: {e}')

    def on_configure(self, state: State):
        self.get_logger().fatal('Configuring...')
        self.replan_count = 0
        self.is_feedback_cleaned_up = False
        self.is_planner_configured = False
        self.success_callback_triggered = False
        try:
            self.test_type_param = self.get_parameter(
                'test_type').get_parameter_value().string_value
            self.strategy_name = self.get_parameter('strategy').get_parameter_value().string_value
            self.current_experiment['method'] = self.strategy_name
            self.timeout = self.get_parameter('timeout_seconds').get_parameter_value().double_value
            self.data_path = self.get_parameter('data_path').get_parameter_value().string_value
            self.filename = f'{self.data_path}/{self.test_type_param}_{self.now.nanoseconds}.csv'

        except Exception as e:
            self.get_logger().error(f'Error getting parameters: {e}')
            return TransitionCallbackReturn.FAILURE

        try:
            self.planner_manager_client = self.create_client(
                ChangeState,
                'planner_node/change_state',
                callback_group=self.callback_group
            )
            self.feedback_manager_client = self.create_client(
                ChangeState,
                'feedback_node/change_state',
                callback_group=self.callback_group
            )

            self.manipulation_sim_client = self.create_client(
                ChangeState,
                'manipulator_simulator/change_state',
                callback_group=self.callback_group
            )
            self.change_pick_order_client = self.create_client(
                TriggerFeedback,
                'set_first_cube',
                callback_group=self.callback_group
            )

            self.on_success_srv = self.create_service(
                Empty,
                'on_success',
                self.on_success_callback,
                # callback_group=self.callback_group
            )
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            self.metrics_timer = self.create_timer(
                self.timeout,
                self.timeout_cb,
                callback_group=self.callback_group,
                autostart=False
            )
            self.replan_count_sub = self.create_subscription(
                Emptymsg,
                'replan_counter',
                self.replan_count_cb,
                10
            )
            self.metrics_timer.cancel()

        except Exception as e:
            self.get_logger().fatal(f'Error creating clients/services/listeners: {e}')
            return TransitionCallbackReturn.FAILURE

        self.init_planner_feedback_simulator()

        if self.test_type_param == 'navigation':
            try:
                self.target_frame = self.get_parameter(
                    f'{self.test_type_param}.target_frame'
                ).get_parameter_value().string_value
                self.collision_min_distance = self.get_parameter(
                    f'{self.test_type_param}.collision_min_distance').get_parameter_value().double_value
                self.collision_max_distance = self.get_parameter(
                    f'{self.test_type_param}.collision_max_distance').get_parameter_value().double_value

                agent_safe_pose = {
                    'x': self.get_parameter(
                        f'{self.test_type_param}.agent_safe_pose.x'
                    ).get_parameter_value().double_value,
                    'y': self.get_parameter(
                        f'{self.test_type_param}.agent_safe_pose.y'
                    ).get_parameter_value().double_value,
                    'min_yaw': self.get_parameter(
                        f'{self.test_type_param}.agent_safe_pose.min_yaw'
                    ).get_parameter_value().double_value
                }

                agent_collision_pose = {
                    'x': self.get_parameter(
                        f'{self.test_type_param}.agent_collision_pose.x'
                    ).get_parameter_value().double_value,
                    'y': self.get_parameter(
                        f'{self.test_type_param}.agent_collision_pose.y'
                    ).get_parameter_value().double_value,
                    'min_yaw': self.get_parameter(
                        f'{self.test_type_param}.agent_collision_pose.min_yaw'
                    ).get_parameter_value().double_value
                }

                self.agent_pose_radius = self.get_parameter(
                    f'{self.test_type_param}.agent_pose_radius').get_parameter_value().double_value

                self.probability_of_colission_param = self.get_parameter(
                    f'{self.test_type_param}.probability_of_colission').get_parameter_value().double_value

                if random.random() < self.probability_of_colission_param:
                    self.agent_pose_to_start = agent_collision_pose
                    self.will_collide = True
                    self.get_logger().fatal('Agent will collide')
                else:
                    self.agent_pose_to_start = agent_safe_pose
                    self.get_logger().fatal('Agent will be safe of collision')

                self.laser_sub = self.create_subscription(
                    LaserScan,
                    'scan_raw',
                    self.laser_cb,
                    10,
                )
            except Exception as e:
                self.get_logger().error(f'Error configuring navigation parameters: {e}')
                return TransitionCallbackReturn.FAILURE
        elif self.test_type_param == 'manipulation':
            try:
                self.probability_of_colission_param = self.get_parameter(
                    f'{self.test_type_param}.probability_of_colission').get_parameter_value().double_value

                if random.random() < self.probability_of_colission_param:
                    self.get_logger().fatal('Arms will collide')
                    self.will_collide = True
                else:
                    self.get_logger().fatal('Arms might be safe of collision')
                self.first_action_sub = self.create_subscription(
                    String,
                    'first_action',
                    self.action_cb,
                    10,
                )
                self.collision_check = self.create_subscription(
                    Bool,
                    'is_colliding',
                    self.check_collision_cb,
                    10,
                )
            except Exception as e:
                self.get_logger().error(f'Error configuring manipulation parameters: {e}')

        self.current_experiment['p_collision'] = self.probability_of_colission_param
        self.current_experiment['will_fail'] = self.will_collide
        self.get_logger().fatal('Successfully configured all nodes.')
        return TransitionCallbackReturn.SUCCESS

    def init_planner_feedback_simulator(self):
        req = ChangeState.Request()
        transition = Transition()
        transition.id = Transition.TRANSITION_CONFIGURE
        req.transition = transition

        # Configure planner and wait for completion
        future = self.planner_manager_client.call_async(req)
        future.add_done_callback(self.planner_configure_cb)
        
        # Configure feedback and activate it after configuration completes
        future_feedback = self.feedback_manager_client.call_async(req)
        future_feedback.add_done_callback(self.feedback_configure_cb)
        
        if self.test_type_param == 'manipulation':
            self.manipulation_sim_client.call_async(req)

    def feedback_configure_cb(self, future):
        """Activate feedback node after successful configuration."""
        if future.result().success:
            self.get_logger().info('Feedback node configured, activating...')
            req = ChangeState.Request()
            transition = Transition()
            transition.id = Transition.TRANSITION_ACTIVATE
            req.transition = transition
            self.feedback_manager_client.call_async(req)
        else:
            self.get_logger().error('Error configuring feedback node, retrying...')
            req = ChangeState.Request()
            transition = Transition()
            transition.id = Transition.TRANSITION_CONFIGURE
            req.transition = transition
            future = self.feedback_manager_client.call_async(req)
            future.add_done_callback(self.feedback_configure_cb)

    def execute_plan(self):
        req = ChangeState.Request()
        transition = Transition()
        transition.id = Transition.TRANSITION_ACTIVATE
        req.transition = transition
        self.get_logger().fatal('Executing plan...')
        if self.test_type_param == 'navigation':
            self.planner_manager_client.call_async(req)
        elif self.test_type_param == 'manipulation' and self.will_collide:
            self.planner_manager_client.call_async(req)
            self.manipulation_sim_client.call_async(req)
        elif self.test_type_param == 'manipulation' and not self.will_collide:
            self.planner_manager_client.call_async(req)
            sleep_duration = Duration(seconds=5.0)
            self.get_clock().sleep_for(sleep_duration)
            self.manipulation_sim_client.call_async(req)

    def change_cube_order(self):
        req = TriggerFeedback.Request()
        req.feedback_input = self.first_action
        self.change_pick_order_client.call_async(req)

    def planner_configure_cb(self, future):
        if future.result().success:
            self.is_planner_configured = True
            self.check_to_start_timer = self.create_timer(
                0.5,
                self.check_to_start,
                callback_group=self.callback_group
            )
        else:
            self.get_logger().error('Error configuring planner node')
            req = ChangeState.Request()
            transition = Transition()
            transition.id = Transition.TRANSITION_CONFIGURE
            req.transition = transition
            future = self.planner_manager_client.call_async(req)
            future.add_done_callback(self.planner_configure_cb)

    def on_activate(self, state: State):
        self.get_logger().fatal('Activating...')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State):
        self.get_logger().fatal('Deactivating...')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State):
        if hasattr(self, 'metrics_timer') and self.metrics_timer is not None:
            self.destroy_timer(self.metrics_timer)
        self.get_logger().fatal('Cleaning up...')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State):
        self.get_logger().fatal('Shutting down...')
        self.save_all_metrics_to_csv()
        return TransitionCallbackReturn.SUCCESS

    def save_all_metrics_to_csv(self):
        """Save all accumulated experiments to a single CSV file."""
        if not self.experiments:
            self.get_logger().warning('No experiments to save')
            return

        metrics_df = pd.DataFrame(self.experiments)
        os.makedirs(self.data_path, exist_ok=True)
        metrics_df.to_csv(self.filename, index=False)
        self.get_logger().fatal(f'Saved {len(self.experiments)} experiments to {self.filename}')

    def save_metrics(self):
        """Save current experiment metrics to the list."""
        now = self.get_clock().now()
        start_time = rclpy.time.Time.from_msg(self.start_test_time)
        duration_sec = (now.nanoseconds - start_time.nanoseconds) / 1e9
        self.current_experiment['duration_time'] = duration_sec
        self.current_experiment['replan_count'] = self.replan_count
        self.experiments.append(self.current_experiment.copy())
        self.get_logger().fatal(f'Experiment {len(self.experiments)} recorded')

        self.reset()

    def feedback_deactivate_cb(self, future):
        """Callback to handle feedback node deactivation response."""
        if future.result().success:
            req_cleanup = ChangeState.Request()
            transition_cleanup = Transition()
            transition_cleanup.id = Transition.TRANSITION_CLEANUP
            req_cleanup.transition = transition_cleanup
            future_feedback = self.feedback_manager_client.call_async(req_cleanup)
            future_feedback.add_done_callback(self.feedback_cleanup_cb)
            self.get_logger().info('Feedback node deactivated successfully')
            return
        else:
            self.get_logger().warn('Feedback node deactivation failed, retrying...')
            req = ChangeState.Request()
            transition = Transition()
            transition.id = Transition.TRANSITION_DEACTIVATE
            req.transition = transition
            future = self.feedback_manager_client.call_async(req)
            future.add_done_callback(self.feedback_deactivate_cb)

    def planner_deactivate_cb(self, future):
        """Handle planner node deactivation response."""
        try:
            if future.result().success:
                req_cleanup = ChangeState.Request()
                transition_cleanup = Transition()
                transition_cleanup.id = Transition.TRANSITION_CLEANUP
                req_cleanup.transition = transition_cleanup
                future_planner = self.planner_manager_client.call_async(req_cleanup)
                future_planner.add_done_callback(self.planner_cleanup_cb)
            else:
                self.get_logger().warn('Planner node deactivation failed, but continuing reset')
                req = ChangeState.Request()
                transition = Transition()
                transition.id = Transition.TRANSITION_DEACTIVATE
                req.transition = transition
                future = self.planner_manager_client.call_async(req)
                future.add_done_callback(self.feedback_deactivate_cb)
        except Exception as e:
            self.get_logger().error(f'Error in planner deactivation callback: {e}')

    def feedback_cleanup_cb(self, future):
        """Handle feedback node cleanup response."""
        if future.result().success:
            self.is_feedback_cleaned_up = True
            self.get_logger().info('Feedback node cleaned up successfully')
        else:
            self.get_logger().warn('Feedback node cleanup failed, retrying...')
            req = ChangeState.Request()
            transition = Transition()
            transition.id = Transition.TRANSITION_CLEANUP
            req.transition = transition
            future = self.feedback_manager_client.call_async(req)
            future.add_done_callback(self.feedback_cleanup_cb)

    def planner_cleanup_cb(self, future):
        """Handle planner node cleanup response."""
        if future.result().success:
            if self.is_feedback_cleaned_up:
                self.on_cleanup(State('inactive', 2))
                self.on_configure(State('unconfigured', 1))
                return
            time.sleep(0.2)
            self.planner_cleanup_cb(future)
        else:
            self.get_logger().warn('Planner node cleanup failed, retrying...')
            req = ChangeState.Request()
            transition = Transition()
            transition.id = Transition.TRANSITION_CLEANUP
            req.transition = transition
            future = self.planner_manager_client.call_async(req)
            future.add_done_callback(self.planner_cleanup_cb)

    def reset(self):
        """Reset the metrics manager and deactivate all managed nodes."""
        self.get_logger().fatal('Resetting metrics manager and deactivating nodes...')
        # Deactivate all nodes - use callbacks to handle failures gracefully
        self.save_all_metrics_to_csv()
        req = ChangeState.Request()
        transition = Transition()
        transition.id = Transition.TRANSITION_DEACTIVATE
        req.transition = transition
        future_feedback = self.feedback_manager_client.call_async(req)
        future_feedback.add_done_callback(self.feedback_deactivate_cb)
        
        future_planner = self.planner_manager_client.call_async(req)
        future_planner.add_done_callback(self.planner_deactivate_cb)

        if self.test_type_param == 'manipulation':
            future_manipulation = self.manipulation_sim_client.call_async(req)
            future_manipulation.add_done_callback(self.manipulation_deactivate_cb)

    def manipulation_deactivate_cb(self, future):
        """Handle manipulation simulator deactivation response."""
        try:
            if future.result().success:
                req_cleanup = ChangeState.Request()
                transition_cleanup = Transition()
                transition_cleanup.id = Transition.TRANSITION_CLEANUP
                req_cleanup.transition = transition_cleanup
                self.manipulation_sim_client.call_async(req_cleanup) # todo check if we have to w8 this also
            else:
                self.get_logger().warn('Manipulation simulator deactivation failed, retrying...')
                req = ChangeState.Request()
                transition = Transition()
                transition.id = Transition.TRANSITION_DEACTIVATE
                req.transition = transition
                future = self.manipulation_sim_client.call_async(req)
                future.add_done_callback(self.manipulation_deactivate_cb)
        except Exception as e:
            self.get_logger().error(f'Error in manipulation deactivation callback: {e}')

    def laser_cb(self, msg):
        if not self.is_checking_for_collision:
            return

        for laser in msg.ranges[200:465]:  # -45 + 45 degrees more or less
            if laser >= self.collision_max_distance or laser <= self.collision_min_distance:
                continue
            self.get_logger().fatal('Collision detected')
            self.is_checking_for_collision = False
            self.current_experiment['success'] = False
            self.save_metrics()
            return

    def check_collision_cb(self, msg):
        if not self.is_checking_for_collision:
            self.get_logger().fatal('Collision check deactivated')

            return
        if msg.data:
            self.get_logger().fatal('Collision detected')
            self.is_checking_for_collision = False
            self.current_experiment['success'] = False
            self.save_metrics()
            return

    def action_cb(self, msg):
        self.first_action = msg.data

    def check_to_start(self):
        self.get_logger().fatal('check_to_start triggered', throttle_duration_sec=5.0)

        if self.test_type_param == 'navigation':
            try:
                trans = self.tf_buffer.lookup_transform(
                    'map', self.target_frame, rclpy.time.Time())
                agent_x = trans.transform.translation.x
                agent_y = trans.transform.translation.y
                qx = trans.transform.rotation.x
                qy = trans.transform.rotation.y
                qz = trans.transform.rotation.z
                qw = trans.transform.rotation.w
                siny_cosp = 2 * (qw * qz + qx * qy)
                cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
                current_yaw = math.atan2(siny_cosp, cosy_cosp)

                dx = agent_x - self.agent_pose_to_start['x']
                dy = agent_y - self.agent_pose_to_start['y']
                min_yaw = self.agent_pose_to_start['min_yaw']
                distance = (dx ** 2 + dy ** 2) ** 0.5

                position_ok = distance <= self.agent_pose_radius
                orientation_ok = math.isclose(current_yaw, min_yaw, abs_tol=0.7)
                
                if position_ok and orientation_ok and self.is_planner_configured:
                    self.is_checking_for_collision = True
                    self.check_to_start_timer.cancel()
                    self.execute_plan()
                    self.start_test_time = self.get_clock().now().to_msg()
                    self.metrics_timer.reset()
            except Exception as e:
                self.get_logger().warning(f'Could not lookup transform: {e}')

        elif self.test_type_param == 'manipulation':
            if not self.first_action:
                return
            self.is_checking_for_collision = True
            self.check_to_start_timer.cancel()
            if self.will_collide:
                self.change_cube_order()
                self.execute_plan()
                self.start_test_time = self.get_clock().now().to_msg()
                self.metrics_timer.reset()
            else:
                cube_colors = ['red',
                               'blue',
                               'green',
                               'yellow',
                               'purple',
                               'cyan',
                               'orange',
                               'black',
                               'grey',]
                random.shuffle(cube_colors)
                start = self.first_action.find('(')
                end = self.first_action.find(')')
                color = self.first_action[start + 1:end].strip()
                idx = cube_colors.index(color)
                new_color = cube_colors[(idx + 1) % len(cube_colors)]
                self.first_action = 'pick(' + new_color + ')'
                self.change_cube_order()
                self.check_to_start_timer.cancel()
                self.execute_plan()
                self.start_test_time = self.get_clock().now().to_msg()
                self.metrics_timer.reset()

    def timeout_cb(self):
        self.get_logger().fatal('Timeout detected')
        self.is_checking_for_collision = False
        self.current_experiment['success'] = False
        self.save_metrics()

    def on_success_callback(self, request, response):
        self.get_logger().fatal('on_success_callback triggered')
        _ = request
        
        # Prevent duplicate success callbacks
        if self.success_callback_triggered:
            self.get_logger().warn('Success callback already triggered, ignoring duplicate')
            return response
        self.success_callback_triggered = True
        
        self.is_checking_for_collision = False
        self.current_experiment['success'] = True
        self.save_metrics()
        return response

    def replan_count_cb(self, msg):
        self.replan_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = MetricsManager(automatically_declare_parameters_from_overrides=True)
    # node.trigger_activate()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    # node.trigger_configure()
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
