import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn as TCR
from rclpy.lifecycle import State
import rclpy.wait_for_message
from rclpy import spin_until_future_complete
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
import time


class FeedbackMode(Enum):
    ONCE = 'once'
    CONTINIOUS = 'continious'


class PlannerNode(LifecycleNode):
    """Lifecycle node for planning in ROS2."""

    def __init__(self, node_name='planner_node', *args, **kwargs):
        """Initialize the PlannerNode lifecycle node."""
        super().__init__(node_name=node_name, *args, **kwargs)

        self.is_activated = False
        self.is_canceled = False
        self.empty_request = Empty.Request()
        # self.executor = MultiThreadedExecutor()
        self.fallback_action = None
        self.last_feedback_result = None
        self.action_manager = None
        self.timer = None
        self.planner_type = None
        self.cb_group = ReentrantCallbackGroup()
        self.success_called = False  # Flag to prevent multiple success calls

    def on_configure(self, state: State):
        self.get_logger().fatal('Configuring...')

        self.image_bridge = CvBridge()

        try:
            self.planner_type = self.get_parameter(
                'planner_type').get_parameter_value().string_value
            self.get_logger().fatal(f'Planner type: {self.planner_type}')
            self.use_image = self.get_parameter(
                self.planner_type + '.use_image').get_parameter_value().bool_value

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
                self.planner_type + '.feedback_mode').get_parameter_value().string_value)
            self.get_logger().fatal(f'feedback mode: {self.feedback_mode}')

            self.system_prompt = self.get_parameter(
                self.planner_type + '.system_prompt').get_parameter_value().string_value

            self.image_topic = self.get_parameter(
                self.planner_type + '.image_topic').get_parameter_value().string_value

            planning_output_name = self.get_parameter(
                self.planner_type + '.output_format').get_parameter_value().string_value

            self.planning_prompt = self.get_parameter(
                self.planner_type + '.planning_prompt').get_parameter_value().string_value

            self.replan_prompt_plugin_names = self.get_parameter(
                self.planner_type + '.replan_prompt_plugins').get_parameter_value().string_array_value

            self.replan_prompt = self.get_parameter(
                self.planner_type + '.replan_prompt').get_parameter_value().string_value

        except Exception as e:
            self.get_logger().error(f'Error loading planner parameters: {e}')
            return TCR.FAILURE

        self.get_logger().fatal('got all params loaded')

        for plugin in self.replan_prompt_plugin_names:
            check_plugin = '{' + plugin + '}'
            if check_plugin not in self.replan_prompt:
                self.get_logger().fatal(
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

        self.action_publisher = self.create_publisher(String,
                                                      'first_action',
                                                      10,
                                                      callback_group=self.cb_group)

        self.llm = BaseClient(vendor=vendor,
                              api_key_variable=api_key_variable_name,
                              model_name=model_name,
                              temperature=temperature,
                              max_tokens=max_tokens)

        self.llm.set_output_format(self.planning_output_format)
        self.llm.set_system_prompt(self.system_prompt)
        if not self.action_manager:
            self.action_manager = BaseAction(backend=action_backend)
        self.action_manager.on_configure()

        if self.use_image:
            topic = self.get_parameter(
                self.planner_type + '.image_topic').get_parameter_value().string_value

            self.image_subscription = self.create_subscription(
                msg_type=ImageMsg,
                topic=topic,
                callback=self.image_callback,
                qos_profile=10,
                callback_group=self.cb_group
            )
            self.get_logger().fatal('sub created')

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
            self.get_logger().fatal('sending prompt with image')

            while (result := self.llm.generate(self.planning_prompt, self.last_image)) is None:
                self.get_logger().error('Error generating the plan, trying again...',
                                        throttle_duration_sec=5)
                time.sleep(0.2)
                continue
            # todo check when is not gemini juandpenan
            josn_result = json.loads(result)
            self.last_plan = josn_result['plan']
            self.feedback_input = str(josn_result['feedback_input'])
            self.fallback_action = str(josn_result['fallback_action'])
            msg = String()
            msg.data = self.last_plan[0]
            self.action_publisher.publish(msg)
            self.get_logger().fatal(f'Initial plan: {josn_result}')
        else:
            self.get_logger().fatal('sending prompt without image')

            while (result := self.llm.generate(self.planning_prompt, self.last_image)) is None:
                self.get_logger().error('Error generating the plan, trying again...',
                                        throttle_duration_sec=5)
                time.sleep(0.2)
                continue
            josn_result = json.loads(result)
            self.last_plan = josn_result['plan']
            self.feedback_input = str(josn_result['feedback_input'])
            self.fallback_action = str(josn_result['fallback_action'])
            msg = String()
            msg.data = self.feedback_input
            self.action_publisher.publish(msg)
            self.get_logger().fatal(f'Initial plan: {josn_result}')

        self.srv = self.create_service(
            TriggerFeedback,
            'cancel_execution',
            self.handle_cancel,
            callback_group=self.cb_group
        )
        self.trigger_client = self.create_client(
            TriggerFeedback,
            'trigger_feedback',
            callback_group=self.cb_group
        )
        self.stop_client = self.create_client(
            Empty,
            'stop_feedback',
            callback_group=self.cb_group
        )
        self.once_client = self.create_client(
            TriggerFeedback,
            'get_feedback_once',
            callback_group=self.cb_group
        )
        self.on_success_client = self.create_client(
            Empty,
            'on_success',
        )

        if self.feedback_mode == FeedbackMode.ONCE:
            if not self.timer:
                self.timer = self.create_timer(
                    1.0,
                    self.once_timer_cb,
                    callback_group=self.cb_group
                )
        elif self.feedback_mode == FeedbackMode.CONTINIOUS:
            if not self.timer:
                self.timer = self.create_timer(
                    1.0,
                    self.continious_timer_cb,
                    callback_group=self.cb_group
                )    
        else:
            self.get_logger().fatal(f'Unknown feedback mode: {self.feedback_mode}')
            return TCR.FAILURE
        self.get_logger().fatal('Successfully configured planner node')
        return TCR.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().fatal('Activating...')
        self.is_activated = True
        return TCR.SUCCESS

    def on_deactivate(self, state: State):
        self.get_logger().fatal('Deactivating...')
        self.is_activated = False
        
        # Cancel timer to prevent new callbacks
        if hasattr(self, 'timer') and self.timer is not None:
            try:
                self.timer.cancel()
            except Exception as e:
                self.get_logger().warn(f'Error cancelling timer during deactivation: {e}')
        
        self.action_manager.cancel_actions()
        time.sleep(0.5)
        self.action_manager.on_deactivate()
        return TCR.SUCCESS

    def on_cleanup(self, state: State):
        self.get_logger().fatal('Cleaning up...')

        self.is_activated = False
        self.is_canceled = False
        self.success_called = False  # Reset the flag for next execution

        if hasattr(self, 'timer') and self.timer is not None:
            try:
                self.timer.cancel()
                self.destroy_timer(self.timer)
                self.timer = None
            except Exception as e:
                self.get_logger().warn(f'Error destroying timer: {e}')

        self.fallback_action = None
        self.last_feedback_result = None
        self.last_image = None
        self.last_plan = None
        self.feedback_input = None
        self.last_action = None

        if hasattr(self, 'srv') and self.srv is not None:
            try:
                self.destroy_service(self.srv)
                self.srv = None
            except Exception as e:
                self.get_logger().warn(f'Error destroying service: {e}')

        if hasattr(self, 'trigger_client') and self.trigger_client is not None:
            try:
                self.destroy_client(self.trigger_client)
                self.trigger_client = None
            except Exception as e:
                self.get_logger().warn(f'Error destroying trigger_client: {e}')

        if hasattr(self, 'stop_client') and self.stop_client is not None:
            try:
                self.destroy_client(self.stop_client)
                self.stop_client = None
            except Exception as e:
                self.get_logger().warn(f'Error destroying stop_client: {e}')

        if hasattr(self, 'once_client') and self.once_client is not None:
            try:
                self.destroy_client(self.once_client)
                self.once_client = None
            except Exception as e:
                self.get_logger().warn(f'Error destroying once_client: {e}')

        if hasattr(self, 'on_success_client') and self.on_success_client is not None:
            try:
                self.destroy_client(self.on_success_client)
                self.on_success_client = None
            except Exception as e:
                self.get_logger().warn(f'Error destroying on_success_client: {e}')

        if hasattr(self, 'image_subscription') and self.image_subscription is not None:
            try:
                self.destroy_subscription(self.image_subscription)
                self.image_subscription = None
            except Exception as e:
                self.get_logger().warn(f'Error destroying image_subscription: {e}')

        if hasattr(self, 'action_publisher') and self.action_publisher is not None:
            try:
                self.destroy_publisher(self.action_publisher)
                self.action_publisher = None
            except Exception as e:
                self.get_logger().warn(f'Error destroying action_publisher: {e}')

        return TCR.SUCCESS

    def on_shutdown(self, state: State):
        self.get_logger().fatal('Shutting down...')
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
        self.get_logger().fatal('Planning', throttle_duration_sec=20)

        self.timer.cancel()

        action = self.last_plan[0]
        # Strip leading numbering like '1. ' from LLM output
        if action and action[0].isdigit() and '. ' in action:
            action = action[action.index('. ') + 2:]
        if 'done' in str(action).lower():
            self.get_logger().fatal('Plan execution complete.')
            # Destroy timer to prevent re-triggering
            if hasattr(self, 'timer') and self.timer is not None:
                self.destroy_timer(self.timer)
                self.timer = None
            req = Empty.Request()
            self.on_success_client.call_async(req)
            return

        self.last_action = action
        if '(' in action and action.endswith(')'):
            action_name = action[:action.index('(')]
            arg = action[action.index('(') + 1:-1]
            self.get_logger().fatal(f'Action name: {action_name}, Arg: {arg}')
            req = TriggerFeedback.Request()
            req.feedback_input = self.feedback_input
            self.trigger_client.call_async(req)
            self.action_manager.execute_action(action_name, arg)
            self.action_manager.wait_for_completition() # check if it only works with navigation then do an if :/
            action_result = self.action_manager.check_for_result()
            self.get_logger().fatal(f'check_for_result returned: {action_result}')

            if action_result == TaskResult.CANCELED:
                self.get_logger().fatal(f'Execution of : {action_name} was cancelled')
                self.stop_client.call_async(self.empty_request)
                if 'forecast' in self.planner_type.lower():
                    action_name = self.fallback_action[:self.fallback_action.index('(')]
                    arg = self.fallback_action[self.fallback_action.index('(') + 1:-1]
                    self.get_logger().fatal('Executing fallback action')
                    self.action_manager.execute_action(action_name, arg)

                replan_copy = self.replan_prompt
                replan_copy = replan_copy.replace('{last_action}', str(self.last_action))
                replan_copy = replan_copy.replace('{feedback_output}',
                                                  str(self.last_feedback_result))
                while (result := self.llm.generate(replan_copy, self.last_image)) is None:
                    self.get_logger().error('Error generating the plan, trying again...',
                                            throttle_duration_sec=5)
                    time.sleep(0.2)
                    continue
                # todo check when is not gemini juandpenan
                josn_result = json.loads(result)
                self.last_plan = josn_result['plan']
                self.get_logger().fatal(f'New plan: {self.last_plan}')
                self.fallback_action = str(josn_result['fallback_action'])
                self.feedback_input = str(josn_result['feedback_input'])
                self.action_manager.wait_for_completition()

            elif action_result == TaskResult.SUCCEEDED:
                self.get_logger().fatal(f'Execution of : {action_name} was sucessfull')
                self.stop_client.call_async(self.empty_request)
                replan_copy = self.replan_prompt
                replan_copy = replan_copy.replace('{last_action}', str(self.last_action))
                replan_copy = replan_copy.replace(
                    '{feedback_output}',
                    str(
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
                self.get_logger().fatal(f'New plan: {self.last_plan}')

            elif action_result == TaskResult.FAILED:
                self.get_logger().fatal(f'There was an error executing: {action_name} ')
                self.stop_client.call_async(self.empty_request)
                replan_copy = self.replan_prompt
                replan_copy = replan_copy.replace('{last_action}', str(self.last_action))
                replan_copy = replan_copy.replace(
                    '{feedback_output}',
                    str(
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
                self.get_logger().fatal(f'New plan: {self.last_plan}')
            
            # Only reset timer if still active and timer still exists
            if self.is_activated and hasattr(self, 'timer') and self.timer is not None:
                try:
                    self.timer.reset()
                except Exception as e:
                    self.get_logger().warn(f'Could not reset timer: {e}')

    def once_timer_cb(self):
        if not self.is_activated:
            return
        if not self.last_image or not self.feedback_input or not self.last_plan:
            return
        self.get_logger().fatal('Planning', throttle_duration_sec=20)

        self.timer.cancel()

        action = self.last_plan[0]
        # Strip leading numbering like '1. ' from LLM output
        if action and action[0].isdigit() and '. ' in action:
            action = action[action.index('. ') + 2:]
        if not self.is_activated:
            self.get_logger().info('Planner deactivated during execution, exiting')
            return

        self.last_action = action
        
        if '(' in action and action.endswith(')'):
            action_name = action[:action.index('(')]
            arg = action[action.index('(') + 1:-1]
            self.get_logger().fatal(f'Action name: {action_name}, Arg: {arg}')
            self.action_manager.execute_action(action_name, arg)
            self.action_manager.wait_for_completition()
            req = TriggerFeedback.Request()
            req.feedback_input = self.feedback_input
            future = self.once_client.call_async(req)
            
            # Wait for the future without blocking the executor
            timeout_sec = 30.0
            start_time = time.time()
            while not future.done():
                time.sleep(0.01)
                if not self.is_activated:
                    self.get_logger().info('Planner deactivated while waiting for feedback')
                    return
                if time.time() - start_time > timeout_sec:
                    self.get_logger().fatal('Timeout waiting for feedback response')
                    return
            
            result = future.result()

            if result is not None and hasattr(result, 'success') and result.success:
                self.get_logger().fatal(f'Execution of : {action_name} was sucessfull')
                replan_copy = self.replan_prompt
                replan_copy = replan_copy.replace('{last_action}', str(self.last_action))
                replan_copy = replan_copy.replace(
                    '{feedback_output}',
                    str(
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
            else:
                self.get_logger().fatal(f'There was an error executing: {action_name} ')
                replan_copy = self.replan_prompt
                replan_copy = replan_copy.replace('{last_action}', str(self.last_action))
                replan_copy = replan_copy.replace(
                    '{feedback_output}',
                    str(
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

        if self.last_plan and 'done' in str(self.last_plan[0]).lower():
            self.get_logger().fatal('Plan execution complete.')
            if hasattr(self, 'timer') and self.timer is not None:
                self.destroy_timer(self.timer)
                self.timer = None

            # Prevent multiple success calls from concurrent executions
            if self.success_called:
                self.get_logger().warn('Success already called, skipping duplicate call')
                return
            self.success_called = True
            
            req = Empty.Request()
            self.get_logger().fatal('Sending on success!!!')
            future = self.on_success_client.call_async(req)
            timeout_sec = 30.0
            start_time = time.time()
            while not future.done():
                time.sleep(0.01)
                if not self.is_activated:
                    self.get_logger().info('Planner deactivated while waiting for feedback')
                    return
                if time.time() - start_time > timeout_sec:
                    self.get_logger().error('Timeout waiting for feedback response')
                    return
            return

        if self.is_activated and hasattr(self, 'timer') and self.timer is not None:
            self.get_logger().info('Triggering next planning cycle!')
            # Call the callback directly instead of resetting the timer
            self.timer.reset()
        else:
            self.get_logger().info('Planner deactivated, stopping execution')

    def handle_cancel(self, req, resp):
        if not self.is_activated:
            return resp
        if self.feedback_mode == FeedbackMode.ONCE:
            return resp
        self.last_feedback_result = req.feedback_input
        self.action_manager.cancel_actions()
        # self.action_manager.set_result(TaskResult.CANCELED)  # Cancelled
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode(
        # allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
