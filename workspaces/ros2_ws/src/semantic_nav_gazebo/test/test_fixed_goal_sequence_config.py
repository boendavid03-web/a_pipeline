from pathlib import Path

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[5]
GOALS_FILE = PROJECT_ROOT / "configs" / "evaluation" / "fixed_four_goals.yaml"


def test_fixed_four_goals_are_reproducible_and_ordered():
    payload = yaml.safe_load(GOALS_FILE.read_text(encoding="utf-8"))
    assert payload["schema"] == "fixed_navigation_goal_suite/v1"
    goals = payload["goals"]
    assert len(goals) == 4
    assert [goal["id"] for goal in goals] == [
        "goal_01",
        "goal_02",
        "goal_03",
        "goal_04",
    ]
    assert all(
        isinstance(goal["x"], (int, float)) and isinstance(goal["y"], (int, float))
        for goal in goals
    )
