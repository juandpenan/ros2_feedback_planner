import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from ros2_feedback_planner.planning.deterministic_plan import feedback_input
from ros2_feedback_planner.planning.deterministic_plan import initial_plan
from ros2_feedback_planner.planning.deterministic_plan import parse_action
from ros2_feedback_planner.planning.deterministic_plan import recovery_action
from ros2_feedback_planner.planning.deterministic_plan import recovery_plan


def test_initial_plan_moves_to_bed_then_done():
    assert initial_plan('bed') == ['move_to(bed)', 'done()']


def test_recovery_plan_backs_up_then_retries_goal():
    assert recovery_plan('bed', 0.5) == [
        'back_up(0.5)',
        'move_to(bed)',
        'done()',
    ]


def test_recovery_action_formats_compact_distance():
    assert recovery_action(1.0) == 'back_up(1)'


def test_feedback_input_names_goal_and_ttc():
    text = feedback_input('bed')
    assert 'move_to(bed)' in text
    assert 'TTC' in text


def test_parse_action_splits_name_and_argument():
    assert parse_action('move_to(bed)') == ('move_to', 'bed')
    assert parse_action('done()') == ('done', None)
