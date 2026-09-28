from pathlib import Path
import sys

import pytest


RUNTIME = Path(__file__).resolve().parents[1] / "runtime"
sys.path.insert(0, str(RUNTIME))

from arena_scenario import load_arena_pedestrians, select_arena_pedestrians  # noqa: E402


SCENARIO = Path(__file__).resolve().parents[1] / "config/arena_eng_lobby_pedestrians.json"


def test_arena_lobby_scenario_has_thirty_ordered_agents_with_a_scared_agent() -> None:
    pedestrians = load_arena_pedestrians(SCENARIO)
    assert len(pedestrians) == 30
    assert pedestrians[0].name == "arena_person_01"
    assert pedestrians[-1].track_id == 30
    assert all(person.waypoint_mode == 1 for person in pedestrians)
    assert all(len(person.route) >= 2 for person in pedestrians)
    assert pedestrians[-1].behavior == "scared"
    assert pedestrians[-1].interaction_distance == 2.0
    assert all(person.behavior == "regular" for person in pedestrians[:-1])
    assert all(person.personal_space > 0.0 for person in pedestrians)


@pytest.mark.parametrize("count", [1, 3, 5, 10, 15, 20, 30])
def test_arena_population_can_scale_from_small_to_large(count: int) -> None:
    selected = select_arena_pedestrians(load_arena_pedestrians(SCENARIO), count)
    assert len(selected) == count
    assert [person.track_id for person in selected] == list(range(1, count + 1))


@pytest.mark.parametrize("count", [0, 31])
def test_arena_population_rejects_out_of_range_counts(count: int) -> None:
    with pytest.raises(ValueError):
        select_arena_pedestrians(load_arena_pedestrians(SCENARIO), count)
