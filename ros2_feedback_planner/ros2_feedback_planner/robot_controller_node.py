"""Single robot controller node for dual manipulator scenarios."""

import rclpy
from rclpy.node import Node
from ros2_feedback_planner.planning.actions import BaseAction
from std_srvs.srv import Trigger
from std_msgs.msg import String, Bool
import sys


class RobotControllerNode(Node):
    """Node that controls a single robot manipulator."""

    def __init__(self, robot_name='robot1'):
        """Initialize robot controller.
        
        Args:
            robot_name: 'robot1' or 'robot2'
        """
        super().__init__(f'{robot_name}_controller')
        self.robot_name = robot_name
        self._current_target = None
        self._last_result = None
        
        self.get_logger().fatal(f'Initializing {robot_name} controller...')
        
        # Create BaseAction for this robot
        self._action = BaseAction(backend='moveit')
        self._action.set_moveit_component_prefix(f'{robot_name}_')
        
        # Subscriber for target cube
        self.target_sub = self.create_subscription(
            String,
            f'/{robot_name}/target_cube',
            self.target_callback,
            10
        )
        
        # Services
        self.configure_srv = self.create_service(
            Trigger,
            f'/{robot_name}/configure',
            self.configure_callback
        )
        
        self.pick_srv = self.create_service(
            Trigger,
            f'/{robot_name}/pick',
            self.pick_callback
        )
        
        self.place_srv = self.create_service(
            Trigger,
            f'/{robot_name}/place',
            self.place_callback
        )
        
        self.get_result_srv = self.create_service(
            Trigger,
            f'/{robot_name}/get_result',
            self.get_result_callback
        )
        
        self.stop_srv = self.create_service(
            Trigger,
            f'/{robot_name}/stop',
            self.stop_callback
        )
        
        # Publisher for collision status
        self.collision_pub = self.create_publisher(
            Bool,
            f'/{robot_name}/is_colliding',
            10
        )
        
        # Timer to check collision at 1 Hz
        self.collision_timer = self.create_timer(1.0, self.check_collision)
        
        self.get_logger().fatal(f'{robot_name} controller ready')
    
    def target_callback(self, msg):
        """Store the target cube for next action."""
        self._current_target = msg.data
        self.get_logger().fatal(f'Target set to: {self._current_target}')
        
    def configure_callback(self, request, response):
        """Configure the robot action."""
        self.get_logger().fatal(f'Configuring {self.robot_name}...')
        try:
            success = self._action.on_configure()
            response.success = success
            response.message = 'Configured successfully' if success else 'Configuration failed'
        except Exception as e:
            response.success = False
            response.message = f'Configuration error: {str(e)}'
        return response
    
    def pick_callback(self, request, response):
        """Execute pick action on current target."""
        if self._current_target is None:
            response.success = False
            response.message = 'No target set'
            return response
            
        self.get_logger().fatal(f'Picking {self._current_target}...')
        try:
            success = self._action.execute_action('pick', self._current_target)
            result = self._action.check_for_result()
            self._last_result = result
            response.success = (result == 1)
            response.message = f'Pick: {result}'
        except Exception as e:
            response.success = False
            response.message = f'Pick error: {str(e)}'
        return response
    
    def place_callback(self, request, response):
        """Execute place action."""
        self.get_logger().fatal('Placing cube...')
        try:
            success = self._action.execute_action('place', None)
            self._action.wait_for_completition()
            result = self._action.check_for_result()
            self._last_result = result
            response.success = (result == 1)  # TaskResult.SUCCEEDED == 1
            response.message = f'Place: {result}'
        except Exception as e:
            response.success = False
            response.message = f'Place error: {str(e)}'
        return response
    
    def get_result_callback(self, request, response):
        """Get last action result."""
        if self._last_result is None:
            response.success = False
            response.message = 'No action executed yet'
        else:
            response.success = (self._last_result == 1)  # TaskResult.SUCCEEDED == 1
            response.message = str(self._last_result)
        return response
    
    def stop_callback(self, request, response):
        """Stop/cancel current execution."""
        self.get_logger().fatal(f'Stop requested for {self.robot_name}')
        try:
            self._action.cancel_actions()
            self._action.on_deactivate()
            response.success = True
            response.message = 'Execution stopped successfully'
            self.get_logger().fatal(f'{self.robot_name} execution cancelled')
        except Exception as e:
            response.success = False
            response.message = f'Stop error: {str(e)}'
            self.get_logger().error(f'Error stopping {self.robot_name}: {str(e)}')
        return response
    
    def check_collision(self):
        """Check for collision and publish status."""
        msg = Bool()
        if self._action.check_arm_collision():
            msg.data = True
            self.collision_pub.publish(msg)
        else:
            msg.data = False
            self.collision_pub.publish(msg)


def main(args=None):
    """Run the robot controller node."""
    rclpy.init(args=args)
    
    # Get robot name from command line args
    robot_name = sys.argv[1] if len(sys.argv) > 1 else 'robot1'
    
    node = RobotControllerNode(robot_name=robot_name)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
