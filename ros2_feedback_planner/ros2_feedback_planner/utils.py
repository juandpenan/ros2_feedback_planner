"""Utility functions for loading plugins in ros2_feedback_planner."""

import importlib
import re
import subprocess
from typing import Optional
from gz.transport13 import Node as GzNode
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.scene_pb2 import Scene

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
    timeout: int = 500,
) -> Optional[PoseStamped]:

    node = GzNode()
    req = Empty()
    success, msg = node.request(f'/world/{world_name}/scene/info',
                                req,
                                Empty,
                                Scene,
                                timeout)
    if not success:
        return None
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    for model in msg.model:
        if entity_name in model.name:
            pose.pose.position.x = float(model.pose.position.x)
            pose.pose.position.y = float(model.pose.position.y)
            pose.pose.position.z = float(model.pose.position.z)

            pose.pose.orientation.x = float(model.pose.orientation.x)
            pose.pose.orientation.y = float(model.pose.orientation.y)
            pose.pose.orientation.z = float(model.pose.orientation.z)
            pose.pose.orientation.w = float(model.pose.orientation.w)
    return pose
