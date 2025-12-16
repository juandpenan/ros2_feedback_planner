"""Scenario coordinator for dual manipulator data generation."""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool, String, Float64
from geometry_msgs.msg import PoseStamped
from ros2_feedback_planner.utils import set_gz_pose
from tf2_ros import TransformListener, Buffer
import itertools
import time
import math


class ScenarioCoordinator(Node):
    """Coordinates dual manipulator scenarios."""

    def __init__(self):
        """Initialize scenario coordinator."""
        super().__init__('scenario_coordinator')
        
        self.get_logger().fatal('Initializing scenario coordinator...')
        
        # Parameters
        cubes = ['grey', 'red', 'blue', 'green', 'yellow',
                 'purple', 'cyan', 'orange', 'black']
        self.declare_parameter('cubes', cubes)
        self.declare_parameter('delay_between_scenarios', 2.0)
        
        for cube in cubes:
            self.declare_parameter(cube, [0.0, 0.0, 0.0])
        
        self._cubes = self.get_parameter('cubes').value
        self._delay = self.get_parameter('delay_between_scenarios').value
        
        # Generate all 81 scenarios (9 cubes x 9 cubes)
        self._scenarios = list(itertools.product(self._cubes, repeat=2))
        self._scenario_index = 0
        
        # State tracking for async execution
        self._robot1_pick_done = False
        self._robot2_pick_done = False
        self._robot1_place_done = False
        self._robot2_place_done = False
        self._robot1_reset_done = False
        self._robot2_reset_done = False
        self._current_cube1 = None
        self._current_cube2 = None
        
        # Collision tracking
        self._robot1_is_colliding = False
        self._robot2_is_colliding = False
        
        # Service clients for robot1
        self.robot1_configure_client = self.create_client(
            Trigger, '/robot1/configure'
        )
        self.robot1_pick_client = self.create_client(
            Trigger, '/robot1/pick'
        )
        self.robot1_place_client = self.create_client(
            Trigger, '/robot1/place'
        )
        self.robot1_stop_client = self.create_client(
            Trigger, '/robot1/stop'
        )
        
        # Service clients for robot2
        self.robot2_configure_client = self.create_client(
            Trigger, '/robot2/configure'
        )
        self.robot2_pick_client = self.create_client(
            Trigger, '/robot2/pick'
        )
        self.robot2_place_client = self.create_client(
            Trigger, '/robot2/place'
        )
        self.robot2_stop_client = self.create_client(
            Trigger, '/robot2/stop'
        )
        
        # Target cube publishers
        self.robot1_target_pub = self.create_publisher(
            String, '/robot1/target_cube', 10
        )
        self.robot2_target_pub = self.create_publisher(
            String, '/robot2/target_cube', 10
        )
        
        # Publishers for scenario metadata and collision status
        self.scenario_metadata_pub = self.create_publisher(
            String, '/scenario_metadata', 10
        )
        self.collision_status_pub = self.create_publisher(
            Bool, '/dual_collision_status', 10
        )
        
        # Publisher for hand distance
        self.hand_distance_pub = self.create_publisher(
            Float64, '/hand_distance', 10
        )
        
        # TF2 setup for distance calculation
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.distance_timer = self.create_timer(0.5, self.publish_hand_distance)
        
        # Subscribers for collision status from both robots
        # self.robot1_collision_sub = self.create_subscription(
        #     Bool,
        #     '/robot1/is_colliding',
        #     self.collision_callback,
        #     10
        # )
        # self.robot2_collision_sub = self.create_subscription(
        #     Bool,
        #     '/robot2/is_colliding',
        #     self.collision_callback,
        #     10
        # )
        
        # Service to start scenarios
        self.start_srv = self.create_service(
            Trigger,
            '/start_scenarios',
            self.start_scenarios_callback
        )
        
        self.get_logger().fatal('Scenario coordinator ready')
    
    def start_scenarios_callback(self, request, response):
        """Start running all scenarios."""
        self.get_logger().fatal('Starting scenario execution...')
        
        # Wait for robot services
        self.get_logger().fatal('Waiting for robot services...')
        services = [
            self.robot1_configure_client,
            self.robot1_pick_client,
            self.robot1_place_client,
            self.robot2_configure_client,
            self.robot2_pick_client,
            self.robot2_place_client
        ]
        for service in services:
            if not service.wait_for_service(timeout_sec=5.0):
                response.success = False
                response.message = f'Service {service.srv_name} not available'
                return response
        
        # Configure both robots
        self.get_logger().fatal('Configuring robots...')
        req = Trigger.Request()
        
        future1 = self.robot1_configure_client.call_async(req)
        future2 = self.robot2_configure_client.call_async(req)
                
        # Start first scenario
        self.execute_next_scenario()
        
        response.success = True
        response.message = f'Started executing {len(self._scenarios)} scenarios'
        return response
    
    def execute_next_scenario(self):
        """Execute the next scenario in the list."""
        if self._scenario_index >= len(self._scenarios):
            self.get_logger().fatal(
                f'Completed all {len(self._scenarios)} scenarios'
            )
            return
        
        cube1, cube2 = self._scenarios[self._scenario_index]
        self._current_cube1 = cube1
        self._current_cube2 = cube2
        
        self.get_logger().fatal(
            f'Scenario {self._scenario_index + 1}/{len(self._scenarios)}: '
            f'Robot1={cube1}, Robot2={cube2}'
        )
        
        # Reset cubes to initial positions
        self.reset_cubes()
        time.sleep(0.5)

        metadata_msg = String()
        metadata_msg.data = (
            f'scenario_{self._scenario_index}:robot1={cube1},robot2={cube2}'
        )
        self.scenario_metadata_pub.publish(metadata_msg)
        
        target1_msg = String()
        target1_msg.data = cube1
        self.robot1_target_pub.publish(target1_msg)
        
        target2_msg = String()
        target2_msg.data = cube2
        self.robot2_target_pub.publish(target2_msg)
        
        time.sleep(0.1)
        
        # Reset pick/place flags
        self._robot1_pick_done = False
        self._robot2_pick_done = False
        self._robot1_place_done = False
        self._robot2_place_done = False
        
        # Execute pick actions in parallel with callbacks
        req = Trigger.Request()
        future1_pick = self.robot1_pick_client.call_async(req)
        future2_pick = self.robot2_pick_client.call_async(req)
        
        future1_pick.add_done_callback(self.robot1_pick_callback)
        future2_pick.add_done_callback(self.robot2_pick_callback)
    
    def robot1_pick_callback(self, future):
        """Callback when robot1 pick completes."""
        try:
            result = future.result()
            self._robot1_pick_done = True
            self.get_logger().fatal(
                f'Robot1 pick completed: {result.success}'
            )
            self.check_pick_completion()
        except Exception as e:
            self.get_logger().error(f'Robot1 pick failed: {str(e)}')
            self._robot1_pick_done = True
            self.check_pick_completion()
    
    def robot2_pick_callback(self, future):
        """Callback when robot2 pick completes."""
        try:
            result = future.result()
            self._robot2_pick_done = True
            self.get_logger().fatal(
                f'Robot2 pick completed: {result.success}'
            )
            self.check_pick_completion()
        except Exception as e:
            self.get_logger().error(f'Robot2 pick failed: {str(e)}')
            self._robot2_pick_done = True
            self.check_pick_completion()
    
    def check_pick_completion(self):
        """Check if both picks are done, then start place actions."""
        if self._robot1_pick_done and self._robot2_pick_done:
            self.get_logger().fatal('Both picks completed, starting place actions')
            
            # Execute place actions in parallel with callbacks
            req = Trigger.Request()
            future1_place = self.robot1_place_client.call_async(req)
            future2_place = self.robot2_place_client.call_async(req)
            
            future1_place.add_done_callback(self.robot1_place_callback)
            future2_place.add_done_callback(self.robot2_place_callback)
    
    def robot1_place_callback(self, future):
        """Callback when robot1 place completes."""
        try:
            result = future.result()
            self._robot1_place_done = True
            self.get_logger().fatal(
                f'Robot1 place completed: {result.success}'
            )
            self.check_place_completion()
        except Exception as e:
            self.get_logger().error(f'Robot1 place failed: {str(e)}')
            self._robot1_place_done = True
            self.check_place_completion()
    
    def robot2_place_callback(self, future):
        """Callback when robot2 place completes."""
        try:
            result = future.result()
            self._robot2_place_done = True
            self.get_logger().fatal(
                f'Robot2 place completed: {result.success}'
            )
            self.check_place_completion()
        except Exception as e:
            self.get_logger().error(f'Robot2 place failed: {str(e)}')
            self._robot2_place_done = True
            self.check_place_completion()
    
    def check_place_completion(self):
        """Check if both places are done, then move to next scenario."""
        if self._robot1_place_done and self._robot2_place_done:
            self.get_logger().fatal('Both places completed')
            
            # Publish collision status (placeholder)
            collision_msg = Bool()
            collision_msg.data = False  # TODO: implement collision detection
            self.collision_status_pub.publish(collision_msg)
            
            self._scenario_index += 1
            
            timer = self.create_timer(self._delay, self._oneshot_timer_callback)
            self._next_scenario_timer = timer
    
    def _oneshot_timer_callback(self):
        """One-shot timer callback to execute next scenario."""
        if self._next_scenario_timer is not None:
            try:
                self._next_scenario_timer.cancel()
            except:
                pass
            try:
                self.destroy_timer(self._next_scenario_timer)
            except:
                pass
            self._next_scenario_timer = None
        self.execute_next_scenario()
    
    def publish_hand_distance(self):
        """Publish the distance between robot1 and robot2 hands."""
        try:
            # Get transform from robot1_hand to robot2_hand (latest available)
            transform = self.tf_buffer.lookup_transform(
                'robot1_hand',
                'robot2_hand',
                rclpy.time.Time(),  # Time(0) = latest available transform
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Calculate Euclidean distance
            translation = transform.transform.translation
            distance = math.sqrt(
                translation.x**2 +
                translation.y**2 +
                translation.z**2
            )
            
            # Publish distance
            msg = Float64()
            msg.data = distance
            self.hand_distance_pub.publish(msg)
            
        except Exception:
            # TF not available yet, skip this iteration
            pass
    
    def collision_callback(self, msg):
        """Handle collision status from robot controllers."""
        is_colliding = msg.data
        
        if is_colliding:
            self.get_logger().fatal('COLLISION DETECTED! Stopping both robots...')
            
            # Reset stop flags
            self._robot1_reset_done = False
            self._robot2_reset_done = False
            
            req = Trigger.Request()
            
            if self.robot1_stop_client.service_is_ready():
                future1 = self.robot1_stop_client.call_async(req)
                # future1.add_done_callback(self.robot1_stop_callback)
            
            if self.robot2_stop_client.service_is_ready():
                future2 = self.robot2_stop_client.call_async(req)
                # future2.add_done_callback(self.robot2_stop_callback)
    
    def robot1_stop_callback(self, future):
        """Callback when robot1 stop completes."""
        try:
            result = future.result()
            self._robot1_reset_done = True
            self.get_logger().fatal(f'Robot1 stopped: {result.success}')
            self.check_stop_completion()
        except Exception as e:
            self.get_logger().error(f'Robot1 stop failed: {str(e)}')
            self._robot1_reset_done = True
            self.check_stop_completion()
    
    def robot2_stop_callback(self, future):
        """Callback when robot2 stop completes."""
        try:
            result = future.result()
            self._robot2_reset_done = True
            self.get_logger().fatal(f'Robot2 stopped: {result.success}')
            self.check_stop_completion()
        except Exception as e:
            self.get_logger().error(f'Robot2 stop failed: {str(e)}')
            self._robot2_reset_done = True
            self.check_stop_completion()
    
    def check_stop_completion(self):
        """Check if both robots stopped, then move to next scenario."""
        if self._robot1_reset_done and self._robot2_reset_done:
            self.get_logger().fatal('Both robots stopped, moving to next scenario')
            
            # Publish collision status
            collision_msg = Bool()
            collision_msg.data = True
            self.collision_status_pub.publish(collision_msg)
            
            # Move to next scenario
            self._scenario_index += 1
            
            # Schedule next scenario after delay
            timer = self.create_timer(self._delay, self._oneshot_timer_callback)
            self._next_scenario_timer = timer
        
    
    def reset_cubes(self):
        """Reset all cubes to their initial positions."""
        for cube in self._cubes:
            position = self.get_parameter(cube).value
            if position and len(position) == 3:
                pose = PoseStamped()
                pose.pose.position.x = position[0]
                pose.pose.position.y = position[1]
                pose.pose.position.z = position[2]
                pose.pose.orientation.w = 1.0
                
                # Gazebo entity names are cube_<color>, not just <color>
                set_gz_pose(f'cube_{cube}', pose)


def main(args=None):
    """Run the scenario coordinator node."""
    rclpy.init(args=args)
    
    node = ScenarioCoordinator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
