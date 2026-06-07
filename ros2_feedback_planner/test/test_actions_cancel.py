import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from ros2_feedback_planner.planning.actions import BaseAction


class FakeGoalHandle:
    def __init__(self):
        self.cancel_goal_async_called = False

    def cancel_goal_async(self):
        self.cancel_goal_async_called = True
        return object()


class FakeNavigator:
    def __init__(self):
        self.result_future = object()
        self.goal_handle = FakeGoalHandle()
        self.cancel_task_called = False

    def cancelTask(self):
        self.cancel_task_called = True

    def info(self, message):
        self.last_info = message


def make_nav_action():
    action = BaseAction.__new__(BaseAction)
    action.use_nav = True
    action.use_moveit = False
    action.navigator = FakeNavigator()
    return action


def test_nonblocking_cancel_uses_async_goal_cancel_without_spinning():
    action = make_nav_action()

    action.cancel_actions(blocking=False)

    assert action.navigator.cancel_task_called is False
    assert action.navigator.goal_handle.cancel_goal_async_called is True
    assert action.nav_cancel_future is not None


def test_blocking_cancel_keeps_existing_navigator_behavior():
    action = make_nav_action()

    action.cancel_actions()

    assert action.navigator.cancel_task_called is True
    assert action.navigator.goal_handle.cancel_goal_async_called is False
