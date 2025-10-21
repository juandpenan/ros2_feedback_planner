import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from sensor_msgs.msg import Image as Imagemsg
from std_msgs.msg import Bool as Boolmsg
from std_msgs.msg import String as Stringmsg
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from ros2_feedback_planner.models.client_base import BaseClient
import asyncio
import threading
import base64
import cv2
import io
import time
import re
from cv_bridge import CvBridge
from google import genai
import os
from std_srvs.srv import Empty
from feedback_planner_interfaces.srv import TriggerFeedback
from PIL import ImageDraw, ImageFont
from enum import Enum
import PIL.Image


class FeedbackNode(LifecycleNode):
    def __init__(self, *args, **kwargs):
        super().__init__('feedback_node', *args, **kwargs)
        self.get_logger().info('FeedbackNode constructed.')
        self.is_executing = False
        self.is_activated = False
        self.last_image = None
        self.last_prompt = None
        self.last_answer = None
        self.probability_threshold = 1.0

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring...')

        self.bridge = CvBridge()

        try:
            self.probability_threshold = self.get_parameter(
                'probability_threshold').get_parameter_value().double_value
            self.vendor = self.get_parameter(
                'llm_client.vendor').get_parameter_value().string_value
            self.api_key_variable_name = self.get_parameter(
                'llm_client.api_key_variable_name').get_parameter_value().string_value
            self.model_name = self.get_parameter(
                'llm_client.model_name').get_parameter_value().string_value
            self.temperature = self.get_parameter(
                'llm_client.temperature').get_parameter_value().double_value
            self.max_tokens = self.get_parameter(
                'llm_client.max_tokens').get_parameter_value().integer_value

            feedback_type = self.get_parameter(
                'feedback_type').get_parameter_value().string_value

            self.forecast_system_prompt = self.get_parameter(
                feedback_type + '.system_prompt').get_parameter_value().string_value
            image_topic = self.get_parameter(
                feedback_type + '.image_topic').get_parameter_value().string_value
            self.forecast_prompt = self.get_parameter(
                feedback_type + '.prompt').get_parameter_value().string_value

        except Exception as e:
            self.get_logger().error(f'Error reading parameters: {e}')
            return TransitionCallbackReturn.FAILURE

        try:
            self.llm = BaseClient(vendor=self.vendor,
                                  api_key_variable=self.api_key_variable_name,
                                  model_name=self.model_name,
                                  temperature=self.temperature,
                                  max_tokens=self.max_tokens)
            self.llm.set_system_prompt(self.forecast_system_prompt)
        except Exception as e:
            self.get_logger().error(f'Error initializing LLM client: {e}')
            return TransitionCallbackReturn.FAILURE

        try:
            cb_group = ReentrantCallbackGroup()

            self.image_sub = self.create_subscription(
                Imagemsg,
                image_topic,
                self.image_callback,
                10,
                callback_group=cb_group)
            
            self.trigger_srv = self.create_service(
                TriggerFeedback,
                'trigger_feedback',
                self.handle_llm_feedback,
                callback_group=cb_group
            )

            self.stop_srv = self.create_service(
                Empty,
                'stop_feedback',
                self.stop_executing,
                callback_group=cb_group
            )

            self.once_srv = self.create_service(
                TriggerFeedback,
                'get_feedback_once',
                self.handle_llm_feedback_once,
                callback_group=cb_group
            )

            self.cancel_action_client = self.create_client(TriggerFeedback,
                                                           'cancel_execution',
                                                           callback_group=cb_group)
            # self.timer = self.create_timer(1.0, self.timer_callback)
        except Exception as e:
            self.get_logger().error(f'Error setting up ros communications {e}')
            return TransitionCallbackReturn.FAILURE
        
        self.get_logger().info('FeedbackNode configured and subscriptions set up.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating...')
        self.is_activated = True
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating...')
        self.is_activated = False
        self.is_executing = False
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up...')
        self.destroy_subscription(self.image_sub)
        self.destroy_service(self.trigger_srv)
        self.destroy_service(self.stop_srv)
        self.destroy_service(self.once_srv)
        self.destroy_client(self.cancel_action_client)
        self.last_image = None
        self.last_prompt = None
        self.last_answer = None

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down...')
        return TransitionCallbackReturn.SUCCESS

    def handle_llm_feedback(self, request, response):
        self.is_executing = True
        self.last_feedback_input = request.feedback_input
        self.get_logger().info(f'New task received: {self.last_feedback_input}')
        prompt_copy = self.forecast_prompt
        self.last_prompt = prompt_copy.replace('{feedback_input}', self.last_feedback_input)
        return response
    
    def stop_executing(self, request, response):
        _ = request
        self.is_executing = False
        return response
    
    def handle_llm_feedback_once(self, request, response):
        response.success = False
        llm = BaseClient(vendor=self.vendor,
                         api_key_variable=self.api_key_variable_name,
                         model_name='gemini-2.5-flash-lite',
                         temperature=self.temperature,
                         max_tokens=self.max_tokens)
        llm.set_system_prompt(self.forecast_system_prompt)
        self.last_feedback_input = request.feedback_input
        self.get_logger().info(f'Handline llm once with input: {self.last_feedback_input}')
        prompt_copy = self.forecast_prompt
        self.last_prompt = prompt_copy.replace('{feedback_input}', self.last_feedback_input)
        answer = llm.generate(prompt=self.last_prompt, image=self.last_image)
        if 'yes' in str(answer).lower():
            response.success = True
        return response

    def image_callback(self, msg):
        self.get_logger().info('Got to the image callback.', once=True)
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        self.last_image = PIL.Image.fromarray(frame_rgb)
        # self.llm.send_rt_input(self.last_prompt, self.last_image)

    # def timer_callback(self):
    #     if not self.is_executing:
    #         return
    #     self.timer.cancel()
     
    #     self.timer.reset()


def strip_markdown_json(text):
    """Remove markdown code fences from JSON response."""
    text = text.strip()
    # Remove ```json and ``` markers
    if text.startswith('```json'):
        text = text[7:]  # Remove ```json
    elif text.startswith('```'):
        text = text[3:]  # Remove ```
    
    if text.endswith('```'):
        text = text[:-3]  # Remove trailing ```
    
    return text.strip()


async def main_loop(node, config):

    async with node.llm.live_session(config) as session:
        while True:
            if node.last_image is None or node.last_prompt is None or not node.is_executing:
                await asyncio.sleep(0.25)
                continue
            # if node.last_image is not None:
            #     node.last_image.show()
            node.get_logger().info(f'Last prompt: {node.last_prompt}')
            await session.send_realtime_input(media=node.last_image)
            await session.send_realtime_input(text=node.last_prompt)
            buffer = ''
            is_canceled = False
            async for response in session.receive():
                if response is not None and response.text is not None:
                    buffer += response.text
                    if 'probability_of_failure' in response.text:
                        match = re.search(r'"probability_of_failure":\s*(\d*\.?\d+)', response.text)
                        if match:
                            prob = float(match.group(1))
                            node.get_logger().info(f'Probability: {prob}')
                            if prob > node.probability_threshold:
                                is_canceled = True
                                break
            node.last_answer = buffer
            if is_canceled:
                req = TriggerFeedback.Request()
                req.feedback_input = node.last_answer
                node.get_logger().info('Canceling current action due to high failure probability')
                node.cancel_action_client.call_async(req)
            await asyncio.sleep(0.25)


def ros_spin_thread(node):
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()


def main(args=None):
    rclpy.init(args=args)
    node = FeedbackNode(automatically_declare_parameters_from_overrides=True)
    node.get_logger().info('Node Constructed')
    ros_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    ros_thread.start()
    config = {'response_modalities': ['TEXT']}

    try:
        while not hasattr(node, 'llm') and not node.is_activated:
            node.get_logger().info('Waiting to be configured and activated',
                                   throttle_duration_sec=5)
            time.sleep(0.5)
        asyncio.run(main_loop(node, config))
    finally:
        node.trigger_deactivate()
        node.trigger_cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
