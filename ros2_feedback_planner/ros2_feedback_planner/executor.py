"""Module for executing plans with feedback in ROS2."""


class PlanExecutor:
    """Executes a sequence of actions with feedback and replanning capabilities."""

    def __init__(self, action_map, logger):
        """
        Initialize the PlanExecutor with action mapping and logging.

        Parameters:
        - action_map: Dict[str, Callable]  e.g. {'move_forward': self.move_forward}
        - feedback_strategy: FeedbackStrategy subclass
        - get_screen_fn: Callable[[], sensor_msgs/Image]
        - logger: e.g. node.get_logger()
        """
        self.actions = action_map
        self.log = logger
        self.current_action = None

    def run_plan(self, plan):
        """Run the plan with feedback checks."""
        while plan:
            raw_action = plan[0]
            action_name, arg = self._parse_action(raw_action)
            if not action_name:
                self.log.error(f'Could not parse action: {raw_action}')
                break
            action_fn = self.actions.get(action_name)
            if not action_fn:
                self.log.error(f'Unknown action: {action_name}')
                break

            success = action_fn(arg)
            if success:
                plan.pop(0)  # Success
            else:
                self.log.warning('Action failed.')

    def _parse_action(self, action_str):
        """Parse an action string and return the action name and argument as a tuple."""
        try:
            if '(' in action_str and action_str.endswith(')'):
                name = action_str[:action_str.index('(')]
                arg = float(action_str[action_str.index('(') + 1:-1])
            return name, arg
        except (ValueError, TypeError) as e:
            self.log.error(f'Failed to parse action: {e}')
        return None, None
