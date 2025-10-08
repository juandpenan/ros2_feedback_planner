import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn as TCR
from rclpy.lifecycle import State
import rclpy.wait_for_message
from ros2_feedback_planner.utils import get_plugin_class
from ros2_feedback_planner.planning.actions import BaseAction, TaskResult
from ros2_feedback_planner.models.client_base import BaseClient
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import String
from rclpy.callback_groups import ReentrantCallbackGroup
import json
from cv_bridge import CvBridge
import cv2
from std_srvs.srv import Empty
import PIL.Image
from feedback_planner_interfaces.srv import TriggerFeedback
from enum import Enum
import threading
import inspect
import time



class FeedbackMode(Enum):
    ONCE = 'once'
    CONTINIOUS = 'continious'


class PlannerNode(LifecycleNode):
    """Lifecycle node for planning in ROS2."""

    def __init__(self, *args, **kwargs):
        """Initialize the PlannerNode lifecycle node."""
        super().__init__(node_name='planner_node', *args, **kwargs)
        # self.declare_parameter('planner_type', rclpy.Parameter.Type.STRING)
        # self.paramet
        self.is_activated = False
        self.is_canceled = False
        self.empty_request = Empty.Request()
        self.executor = MultiThreadedExecutor()

    def on_configure(self, state: State):
        self.get_logger().info('Configuring...')

        self.image_bridge = CvBridge()

        try:
            planner_type = self.get_parameter(
                'planner_type').get_parameter_value().string_value
            self.get_logger().info(f'Planner type: {planner_type}')
            self.use_image = self.get_parameter(
                planner_type + '.use_image').get_parameter_value().bool_value

            vendor = self.get_parameter(
                'llm_client.vendor').get_parameter_value().string_value
            api_key_variable_name = self.get_parameter(
                'llm_client.api_key_variable_name').get_parameter_value().string_value
            model_name = self.get_parameter(
                'llm_client.model_name').get_parameter_value().string_value
            temperature = self.get_parameter(
                'llm_client.temperature').get_parameter_value().double_value
            max_tokens = self.get_parameter(
                'llm_client.max_tokens').get_parameter_value().integer_value
            action_backend = self.get_parameter(
                'action_backend').get_parameter_value().string_value

            self.feedback_mode = FeedbackMode(self.get_parameter(
                planner_type + '.feedback_mode').get_parameter_value().string_value)
            self.system_prompt = self.get_parameter(
                planner_type + '.system_prompt').get_parameter_value().string_value

            self.image_topic = self.get_parameter(
                planner_type + '.image_topic').get_parameter_value().string_value

            planning_output_name = self.get_parameter(
                planner_type + '.output_format').get_parameter_value().string_value

            self.planning_prompt = self.get_parameter(
                planner_type + '.planning_prompt').get_parameter_value().string_value

            self.replan_prompt_plugin_names = self.get_parameter(
                planner_type + '.replan_prompt_plugins').get_parameter_value().string_array_value

            self.replan_prompt = self.get_parameter(
                planner_type + '.replan_prompt').get_parameter_value().string_value

        except Exception as e:
            self.get_logger().error(f'Error loading planner parameters: {e}')
            return TCR.FAILURE

        self.get_logger().info('got all params loaded')

        for plugin in self.replan_prompt_plugin_names:
            check_plugin = '{' + plugin + '}'
            if check_plugin not in self.replan_prompt:
                self.get_logger().info(
                    f'Failed to find plugin {plugin} in replan prompt. '
                    'Please check the param!'
                )
                return TCR.FAILURE
        try:
            self.planning_output_format = get_plugin_class(
                'ros2_feedback_planner.planning.planning_output_formats',
                planning_output_name
            )
            self.get_logger().info('output format loaded')
        except Exception as e:
            self.get_logger().error(f'Unexpected error in planning output format plugin: {e}')
            return TCR.FAILURE
        
        cb_group = ReentrantCallbackGroup()

        self.action_publisher = self.create_publisher(String,
                                                      'first_action',
                                                      10,
                                                      callback_group=cb_group)

        self.llm = BaseClient(vendor=vendor,
                              api_key_variable=api_key_variable_name,
                              model_name=model_name,
                              temperature=temperature,
                              max_tokens=max_tokens)

        self.llm.set_output_format(self.planning_output_format)
        self.llm.set_system_prompt(self.system_prompt)
        self.action_manager = BaseAction(backend=action_backend)


        if self.use_image:
            topic = self.get_parameter(
                planner_type + '.image_topic').get_parameter_value().string_value

            self.image_subscription = self.create_subscription(
                msg_type=ImageMsg,
                topic=topic,
                callback=self.image_callback,
                qos_profile=10,
                callback_group=cb_group
            )
            self.get_logger().info('sub created')

            try:
                was_received, image = rclpy.wait_for_message.wait_for_message(
                    ImageMsg,
                    self,
                    topic
                )
                frame_rgb = cv2.cvtColor(self.image_bridge.imgmsg_to_cv2(image,
                                                                         desired_encoding='bgr8'),
                                         cv2.COLOR_BGR2RGB)
                self.last_image = PIL.Image.fromarray(frame_rgb)
            except Exception as e:
                self.get_logger().error(f'Error waiting for image message: {e}')
                return TCR.FAILURE
            if not was_received:
                self.get_logger().error('No image received on topic: {}'.format(topic))
                return TCR.FAILURE

        if self.use_image and self.last_image:
            self.get_logger().info('sending prompt with image')
            result = self.llm.generate(self.planning_prompt, self.last_image)
            # todo check when is not gemini juandpenan
            josn_result = json.loads(result)
            self.last_plan = josn_result['plan']
            self.feedback_input = str(josn_result['feedback_input'])
            msg = String()
            msg.data = self.feedback_input
            self.action_publisher.publish(msg)
            self.get_logger().info(f'Initial plan: {josn_result}')
        else:
            # result = SimpleNamespace(**self.llm.generate(self.planning_prompt))
            result = self.llm.generate(self.planning_prompt)
            josn_result = json.loads(result)
            self.last_plan = josn_result['plan']
            self.feedback_input = str(josn_result['feedback_input'])
            msg = String()
            msg.data = self.feedback_input
            self.action_publisher.publish(msg)
            self.get_logger().info(f'Initial plan: {josn_result}')

        self.srv = self.create_service(
            TriggerFeedback,
            'cancel_execution',
            self.handle_cancel,
            callback_group=cb_group
        )
        self.trigger_client = self.create_client(
            TriggerFeedback,
            'trigger_feedback',
            callback_group=cb_group
        )
        self.stop_client = self.create_client(
            Empty,
            'stop_feedback',
            callback_group=cb_group
        )
        self.once_client = self.create_client(
            TriggerFeedback,
            'get_feedback_once',
            callback_group=cb_group
        )
        self.on_success_client = self.create_client(
            Empty,
            'on_success',
            callback_group=cb_group
        )

        if self.feedback_mode == FeedbackMode.ONCE:
            self.timer = self.create_timer(
                1.0,
                self.once_timer_cb,
                callback_group=cb_group
            )
        elif self.feedback_mode == FeedbackMode.CONTINIOUS:
            self.timer = self.create_timer(
                1.0,
                self.continious_timer_cb,
                callback_group=cb_group
            )
        else:
            self.get_logger().error(f'Unknown feedback mode: {self.feedback_mode}')
            return TCR.FAILURE
        return TCR.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info('Activating...')
        self.is_activated = True
        return TCR.SUCCESS

    def on_deactivate(self, state: State):
        self.get_logger().info('Deactivating...')
        self.action_manager.cancel_actions()
        return TCR.SUCCESS

    def on_cleanup(self, state: State):
        self.get_logger().info('Cleaning up...')

        self.is_activated = False
        self.is_canceled = False
        self.empty_request = Empty.Request()
        self.executor = MultiThreadedExecutor()

        keep_attrs = {'is_activated', 'is_canceled', 'empty_request', 'executor'}
        for attr in list(self.__dict__.keys()):
            if attr not in keep_attrs:
                delattr(self, attr)

        return TCR.SUCCESS

    def on_shutdown(self, state: State):
        self.get_logger().info('Shutting down...')
        return TCR.SUCCESS

    def image_callback(self, msg):
        if not self.is_activated:
            return
        cv_image = self.image_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        self.last_image = PIL.Image.fromarray(frame_rgb)

    def continious_timer_cb(self):
        if not self.is_activated:
            return
        if not self.last_image or not self.feedback_input or not self.last_plan:
            return
        self.get_logger().info('Planning', throttle_duration_sec=20)

        self.timer.cancel()

        action = self.last_plan[0]
        if 'done' in str(action).lower():
            self.get_logger().info('Plan execution complete.')
            req = Empty.Request()
            future = self.on_success_client.call_async(req)
            self.executor.spin_until_future_complete(future=future)
            return

        self.last_action = action
        if '(' in action and action.endswith(')'):
            action_name = action[:action.index('(')]
            arg = action[action.index('(') + 1:-1]
            self.get_logger().info(f'Action name: {action_name}, Arg: {arg}')
            req = TriggerFeedback.Request()
            req.feedback_input = self.feedback_input
            self.trigger_client.call_async(req)
            self.action_manager.execute_action(action_name, arg)
            self.action_manager.wait_for_completition()

            result = self.action_manager.check_for_result()
            self.get_logger().info(f'check_for_result returned: {result}')
            print(f'check_for_result returned: {result}')

            if self.action_manager.check_for_result() == TaskResult.SUCCEEDED:
                self.get_logger().info(f'Execution of : {action_name} was sucessfull')
                self.stop_client.call_async(self.empty_request)
                replan_copy = self.replan_prompt
                replan_copy.replace('{last_action}', self.last_action)
                replan_copy.replace(
                    '{feedback_output}',
                    (
                        'The action was successfully executed'
                    )
                )
                while (result := self.llm.generate(replan_copy, self.last_image)) is None:
                    self.get_logger().error('Error generating the plan, trying again...',
                                            throttle_duration_sec=5)
                    time.sleep(0.2)
                    continue
                # todo check when is not gemini juandpenan
                josn_result = json.loads(result)
                self.last_plan = josn_result['plan']
                self.feedback_input = str(josn_result['feedback_input'])
                self.get_logger().info(f'New plan: {self.last_plan}')
            elif self.action_manager.check_for_result() == TaskResult.FAILED:
                self.get_logger().info(f'There was an error executing: {action_name} ')
                self.stop_client.call_async(self.empty_request)
                replan_copy = self.replan_prompt
                replan_copy.replace('{last_action}', self.last_action)
                replan_copy.replace(
                    '{feedback_output}',
                    (
                        'There was an error executing the action'
                    )
                )
                result = self.llm.generate(replan_copy, self.last_image)
                # todo check when is not gemini juandpenan
                josn_result = json.loads(result)
                self.last_plan = josn_result['plan']
                self.feedback_input = str(josn_result['feedback_input'])
                self.get_logger().info(f'New plan: {self.last_plan}')

        self.timer.reset()

    def completition_cb(self, result):
        pass

    def once_timer_cb(self):
        if not self.is_activated:
            return
        if not self.last_image or not self.feedback_input or not self.last_plan:
            return
        self.get_logger().info('Planning', throttle_duration_sec=20)

        self.timer.cancel()

        action = self.last_plan[0]
        if 'done' in str(action).lower():
            self.get_logger().info('Plan execution complete.')
            req = Empty.Request()
            future = self.on_success_client.call_async(req)
            self.executor.spin_until_future_complete(future=future)
            return

        self.last_action = action
        if '(' in action and action.endswith(')'):
            action_name = action[:action.index('(')]
            arg = action[action.index('(') + 1:-1]
            self.get_logger().info(f'Action name: {action_name}, Arg: {arg}')
            self.action_manager.execute_action(action_name, arg)
            self.action_manager.wait_for_completition()
            req = TriggerFeedback.Request()
            req.feedback_input = self.feedback_input
            future = self.once_client.call_async(req)
            self.executor.spin_until_future_complete(future=future)
            result = future.result()

            if result is not None and hasattr(result, 'success') and result.success:
                self.get_logger().info(f'Execution of : {action_name} was sucessfull')
                replan_copy = self.replan_prompt
                replan_copy.replace('{last_action}', self.last_action)
                replan_copy.replace(
                    '{feedback_output}',
                    (
                        'The action was successfully executed'
                    )
                )
                while (result := self.llm.generate(replan_copy, self.last_image)) is None:
                    self.get_logger().error('Error generating the plan, trying again...',
                                            throttle_duration_sec=5)
                    time.sleep(0.2)
                    continue
                josn_result = json.loads(result)
                self.last_plan = josn_result['plan']
                self.feedback_input = str(josn_result['feedback_input'])
                self.get_logger().info(f'New plan: {self.last_plan}')
                result = self.llm.generate(replan_copy, self.last_image)
            else:
                self.get_logger().info(f'There was an error executing: {action_name} ')
                replan_copy = self.replan_prompt
                replan_copy.replace('{last_action}', self.last_action)
                replan_copy.replace(
                    '{feedback_output}',
                    (
                        'There was an error executing the action'
                    )
                )
                while (result := self.llm.generate(replan_copy, self.last_image)) is None:
                    self.get_logger().error('Error generating the plan, trying again...',
                                            throttle_duration_sec=5)
                    time.sleep(0.2)
                    continue
                # todo check when is not gemini juandpenan
                josn_result = json.loads(result)
                self.last_plan = josn_result['plan']
                self.feedback_input = str(josn_result['feedback_input'])
                self.get_logger().info(f'New plan: {self.last_plan}')

        self.timer.reset()

    def handle_cancel(self, req, resp):
        if not self.is_activated:
            return resp
        if self.feedback_mode == FeedbackMode.ONCE:
            return resp

        self.get_logger().info('Action was cancalled')
        self.action_manager.cancel_actions()
        replan_copy = self.replan_prompt
        replan_copy.replace('{last_action}', self.last_action)
        replan_copy.replace(
            '{feedback_output}', req.feedback_input)

        while (result := self.llm.generate(replan_copy, self.last_image)) is None:
            self.get_logger().error('Error generating the plan, trying again...',
                                    throttle_duration_sec=5)
            time.sleep(0.2)
            continue
        # todo check when is not gemini juandpenan
        josn_result = json.loads(result)
        self.last_plan = josn_result['plan']
        self.get_logger().info(f'New plan: {self.last_plan}')
        self.feedback_input = str(josn_result['feedback_input'])
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode(
        # allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
