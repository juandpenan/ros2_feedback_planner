"""Deterministic navigation plan helpers used for API-free experiments."""


def format_distance(distance_m):
    """Format distances compactly for action strings."""
    return f'{distance_m:g}'


def initial_plan(destination='bed'):
    """Return the fixed navigation plan."""
    return [f'move_to({destination})', 'done()']


def recovery_action(backup_distance_m=0.5):
    """Return the fixed TTC recovery action."""
    return f'back_up({format_distance(backup_distance_m)})'


def recovery_plan(destination='bed', backup_distance_m=0.5):
    """Return the fixed recovery plan after TTC cancellation."""
    return [recovery_action(backup_distance_m), *initial_plan(destination)]


def feedback_input(destination='bed'):
    """Return a stable feedback description for TTC monitoring logs."""
    return (
        f'Action move_to({destination}) should keep path_clear true. '
        'Cancel if laser TTC predicts collision within the configured horizon.'
    )


def parse_action(action):
    """Parse action strings like move_to(bed) into name and argument."""
    if '(' not in action or not action.endswith(')'):
        raise ValueError(f'Invalid action string: {action}')
    name = action[:action.index('(')]
    arg = action[action.index('(') + 1:-1]
    return name, arg or None
