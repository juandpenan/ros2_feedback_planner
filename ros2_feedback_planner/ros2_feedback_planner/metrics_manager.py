import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import rclpy.parameter
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from std_srvs.srv import Empty
from feedback_planner_interfaces.srv import TriggerFeedback
import pandas as pd
from tf2_ros import Buffer, TransformListener
import random
import math
import os


class MetricsManager(LifecycleNode):
    """Lifecycle node for managing metrics subscriptions and callbacks."""

    def __init__(self, *args, **kwargs):
        """Initialize the MetricsManager node and its resources."""
        super().__init__('metrics_manager_node', *args, **kwargs)
        self.callback_group = ReentrantCallbackGroup()
        self.subscription = None
        self.first_action = None
        self.will_collide = False
        self.is_feedback_configured = False
        self.metrics_df = pd.DataFrame(columns=['name',
                                                'duration_time',
                                                'p_collision',
                                                'success'])
        self.start_test_time = None
        self.is_checking_for_collision = False

    def on_configure(self, state: State):
        self.get_logger().info('Configuring...')

        try:
            self.test_type_param = self.get_parameter('test_type').get_parameter_value().string_value
            self.metrics_df.loc[0, 'name'] = self.test_type_param
            self.strategy_name = self.get_parameter('strategy').get_parameter_value().string_value
            self.timeout = self.get_parameter('timeout_seconds').get_parameter_value().double_value
            self.data_path = self.get_parameter('data_path').get_parameter_value().string_value
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
                callback_group=self.callback_group
            )
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            self.metrics_timer = self.create_timer(
                self.timeout,
                self.timeout_cb,
                callback_group=self.callback_group,
                autostart=False
            )
            self.metrics_timer.cancel()

        except Exception as e:
            self.get_logger().error(f'Error creating clients/services/listeners: {e}')
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

                self.metrics_df.loc[0, 'p_collision'] = self.probability_of_colission_param

                if random.random() < self.probability_of_colission_param:
                    self.agent_pose_to_start = agent_collision_pose
                    self.get_logger().info('Agent will collide')
                else:
                    self.agent_pose_to_start = agent_safe_pose
                    self.get_logger().info('Agent will be safe of collision')

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

                self.metrics_df.loc[0, 'p_collision'] = self.probability_of_colission_param

                if random.random() < self.probability_of_colission_param:
                    self.get_logger().info('Arms will collide')
                    self.will_collide = True
                else:
                    self.get_logger().info('Arms might be safe of collision')
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

        self.get_logger().info('Successfully configured all nodes.')
        return TransitionCallbackReturn.SUCCESS

    def init_planner_feedback_simulator(self):
        req = ChangeState.Request()
        transition = Transition()
        transition.id = Transition.TRANSITION_CONFIGURE
        req.transition = transition

        future = self.planner_manager_client.call_async(req)
        future.add_done_callback(self.planner_configure_cb)
        self.feedback_manager_client.call_async(req)
        self.manipulation_sim_client.call_async(req)

        # acticate feedback node
        req = ChangeState.Request()
        transition = Transition()
        transition.id = Transition.TRANSITION_ACTIVATE
        req.transition = transition
        self.feedback_manager_client.call_async(req)

    def execute_plan(self):
        req = ChangeState.Request()
        transition = Transition()
        transition.id = Transition.TRANSITION_ACTIVATE
        req.transition = transition
        self.get_logger().info('Executing plan...')
        if self.test_type_param == 'navigation':
            self.planner_manager_client.call_async(req)
        elif self.test_type_param == 'manipulation' and self.will_collide:
            self.manipulation_sim_client.call_async(req)
            self.planner_manager_client.call_async(req)
        elif self.test_type_param == 'manipulation' and not self.will_collide:
            self.planner_manager_client.call_async(req)
            self.get_clock().sleep_for(0.5)
            self.manipulation_sim_client.call_async(req)

    def change_cube_order(self):
        req = TriggerFeedback.Request()
        req.feedback_input = self.first_action
        self.change_pick_order_client.call_async(req)

    def planner_configure_cb(self, future):
        if future.result().success:
            self.check_to_start_timer = self.create_timer(
                0.5,
                self.check_to_start,
                callback_group=self.callback_group
            )
        else:
            req = ChangeState.Request()
            transition = Transition()
            transition.id = Transition.TRANSITION_CONFIGURE
            req.transition = transition
            future = self.planner_manager_client.call_async(req)
            future.add_done_callback(self.planner_configure_cb)

    def on_activate(self, state: State):
        self.get_logger().info('Activating...')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State):
        self.get_logger().info('Deactivating...')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State):
        self.get_logger().info('Cleaning up...')
        if self.subscription is not None:
            self.destroy_subscription(self.subscription) 
            self.subscription = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State):
        self.get_logger().info('Shutting down...')
        return TransitionCallbackReturn.SUCCESS

    def laser_cb(self, msg):
        if not self.is_checking_for_collision:
            return

        for laser in msg.ranges[200:465]:  # -45 + 45 degrees more or less 
            if laser >= self.collision_max_distance or laser <= self.collision_min_distance:
                continue
            self.get_logger().info('Collision detected')
            self.is_checking_for_collision = False
            self.metrics_df.loc[0, 'success'] = False
            now = self.get_clock().now()
            start_time = rclpy.time.Time.from_msg(self.start_test_time)
            duration_sec = (now.nanoseconds - start_time.nanoseconds) / 1e9
            self.metrics_df.loc[0, 'duration_time'] = duration_sec
            name = self.metrics_df['name'].iloc[0]
            filename = f'{self.data_path}/{name}_{now.nanoseconds}.csv'
            os.makedirs(self.data_path, exist_ok=True)
            self.metrics_df.to_csv(filename, index=False)
            return
        #  todo(juandpenan) self.restart()

    def check_collision_cb(self, msg):
        if msg.data:
            self.get_logger().info('Collision detected')
            self.is_checking_for_collision = False
            self.metrics_df.loc[0, 'success'] = False
            now = self.get_clock().now()
            start_time = rclpy.time.Time.from_msg(self.start_test_time)
            duration_sec = (now.nanoseconds - start_time.nanoseconds) / 1e9
            self.metrics_df.loc[0, 'duration_time'] = duration_sec
            name = self.metrics_df['name'].iloc[0]
            filename = f'{self.data_path}/{name}_{now.nanoseconds}.csv'
            os.makedirs(self.data_path, exist_ok=True)
            self.metrics_df.to_csv(filename, index=False)
            self.destroy_subscription(self.collision_check)
            return

    def action_cb(self, msg):
        self.first_action = msg.data

    def check_to_start(self):
        self.get_logger().info('check_to_start triggered', throttle_duration_sec=5.0)
        if not self.first_action:
            return

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

                if distance <= self.agent_pose_radius and math.isclose(current_yaw, min_yaw, abs_tol=0.7):
                    self.is_checking_for_collision = True
                    self.check_to_start_timer.cancel()
                    self.execute_plan()
                    self.start_test_time = self.get_clock().now().to_msg()
                    self.metrics_timer.reset()
            except Exception as e:
                self.get_logger().warning(f'Could not lookup transform: {e}')
        elif self.test_type_param == 'manipulation':
            if self.will_collide:
                self.change_cube_order()
                self.check_to_start_timer.cancel()
                self.execute_plan()
                self.start_test_time = self.get_clock().now().to_msg()
                self.metrics_timer.reset()
            else:
                self.check_to_start_timer.cancel()
                self.execute_plan()
                self.start_test_time = self.get_clock().now().to_msg()
                self.metrics_timer.reset()

    def timeout_cb(self):
        self.get_logger().info('Timeout callback triggered: test failed or timed out.')
        self.metrics_df.loc[0, 'success'] = False
        now = self.get_clock().now()
        start_time = rclpy.time.Time.from_msg(self.start_test_time)
        duration_sec = (now.nanoseconds - start_time.nanoseconds) / 1e9
        self.metrics_df.loc[0, 'duration_time'] = duration_sec
        name = self.metrics_df['name'].iloc[0]
        filename = f'{self.data_path}/{name}_{now.nanoseconds}.csv'
        self.metrics_df.to_csv(filename, index=False)

    def on_success_callback(self, request, response):
        self.get_logger().info('on_success_callback triggered')
        _ = request
        self.metrics_df.loc[0, 'success'] = True
        now = self.get_clock().now()
        start_time = rclpy.time.Time.from_msg(self.start_test_time)
        duration_sec = (now.nanoseconds - start_time.nanoseconds) / 1e9
        self.metrics_df.loc[0, 'duration_time'] = duration_sec
        name = self.metrics_df['name'].iloc[0]
        filename = f'{self.data_path}/{name}_{now.nanoseconds}.csv'
        self.metrics_df.to_csv(filename, index=False)
        return response


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
