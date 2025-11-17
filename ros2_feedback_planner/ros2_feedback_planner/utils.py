"""Utility functions for loading plugins in ros2_feedback_planner."""

import importlib
import re
import subprocess
from typing import Optional
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_v_pb2 import Pose_V
import time

from geometry_msgs.msg import PoseStamped


def load_plugin(module_path: str, class_name: str, **kwargs):
    module = importlib.import_module(module_path)
    klass = getattr(module, class_name)
    return klass(**kwargs)


def get_plugin_class(module_path: str, class_name: str):
    module = importlib.import_module(module_path)
    return getattr(module, class_name)


def get_gz_pose(
    entity_name: str,
    world_name: str = 'default',
    frame_id: str = 'world',
    timeout: int = 5000,
) -> Optional[PoseStamped]:
    """Get the current dynamic pose of an entity from Gazebo.

    Uses the /world/{world_name}/dynamic_pose/info topic to get real-time poses.

    Args:
        entity_name: Name of the entity to find
        world_name: Name of the Gazebo world
        frame_id: Frame ID for the PoseStamped message
        timeout: Timeout in milliseconds

    Returns:
        PoseStamped with current pose or None if not found
    """
    node = GzNode()
    result = {'pose': None}

    # Subscribe to dynamic pose topic with lambda callback
    topic = f'/world/{world_name}/dynamic_pose/info'
    node.subscribe(
        Pose_V,
        topic,
        lambda msg: _process_pose_message(msg, entity_name, frame_id, result)
    )

    # Wait for message with timeout
    start_time = time.time()
    timeout_sec = timeout / 1000.0

    while result['pose'] is None and (time.time() - start_time) < timeout_sec:
        time.sleep(0.01)  # 10ms sleep to avoid busy waiting

    return result['pose']


def _process_pose_message(msg, entity_name: str, frame_id: str, result: dict):
    """Process dynamic pose messages and extract entity pose.

    Args:
        msg: Pose_V message from Gazebo
        entity_name: Name of the entity to find
        frame_id: Frame ID for the PoseStamped message
        result: Dictionary to store the result pose
    """
    for pose_msg in msg.pose:
        if entity_name in pose_msg.name:
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = float(pose_msg.position.x)
            pose.pose.position.y = float(pose_msg.position.y)
            pose.pose.position.z = float(pose_msg.position.z)
            pose.pose.orientation.x = float(pose_msg.orientation.x)
            pose.pose.orientation.y = float(pose_msg.orientation.y)
            pose.pose.orientation.z = float(pose_msg.orientation.z)
            pose.pose.orientation.w = float(pose_msg.orientation.w)
            result['pose'] = pose
            return


def set_gz_pose(
    entity_name: str,
    target_pose: PoseStamped,
    world_name: str = 'default',
    timeout: int = 5000,
) -> bool:
    """Set the pose of an entity in Gazebo.

    Uses the /world/{world_name}/set_pose service to set entity poses.

    Args:
        entity_name: Name of the entity to set pose for
        target_pose: PoseStamped with the desired pose
        world_name: Name of the Gazebo world
        timeout: Timeout in milliseconds

    Returns:
        True if pose was set successfully, False otherwise
    """
    node = GzNode()

    # Prepare the pose message
    pose_msg = Pose()
    pose_msg.name = entity_name
    pose_msg.position.x = target_pose.pose.position.x
    pose_msg.position.y = target_pose.pose.position.y
    pose_msg.position.z = target_pose.pose.position.z
    pose_msg.orientation.x = target_pose.pose.orientation.x
    pose_msg.orientation.y = target_pose.pose.orientation.y
    pose_msg.orientation.z = target_pose.pose.orientation.z
    pose_msg.orientation.w = target_pose.pose.orientation.w

    # Call the service
    service_name = f'/world/{world_name}/set_pose'
    success = node.request(service_name, pose_msg, Pose, Boolean, timeout)

    return success


def is_on_table(cube_name: str) -> bool:
    """
    Check if a cube is within the table boundaries with ±2 cm tolerance.

    Table boundaries (from world file):
    X: 0.65 to 0.95
    Y: -0.25 to 0.25
    Z: ~0.53-0.55 (table height)

    Args:
        cube_name: Name of the cube to check (e.g., 'red', 'blue')

    Returns:
        True if cube is within table boundaries (±2 cm), False otherwise
    """
    # Table boundaries with ±2 cm (0.02 m) tolerance
    x_min = 0.65 - 0.02
    x_max = 0.95 + 0.02
    y_min = -0.25 - 0.02
    y_max = 0.25 + 0.02
    z_min = 0.50  # Slightly below table surface
    z_max = 0.60  # Slightly above initial cube height

    # Get pose from Gazebo
    pose = get_gz_pose(f'cube_{cube_name}', world_name='default')

    if pose is None:
        print(f'Could not get pose for cube_{cube_name}')
        return False

    x = pose.pose.position.x
    y = pose.pose.position.y
    z = pose.pose.position.z

    # Check if within boundaries
    x_in_range = x_min <= x <= x_max
    y_in_range = y_min <= y <= y_max
    z_in_range = z_min <= z <= z_max
    is_within = x_in_range and y_in_range and z_in_range

    if not is_within:
        print(
            f'Cube {cube_name} is OUT OF BOUNDS: '
            f'pos=({x:.3f}, {y:.3f}, {z:.3f}), '
            f'expected X:[{x_min:.2f}, {x_max:.2f}], '
            f'Y:[{y_min:.2f}, {y_max:.2f}], '
            f'Z:[{z_min:.2f}, {z_max:.2f}]'
        )

    return is_within
