from pathlib import Path
import sys


RUNTIME = Path(__file__).resolve().parents[1] / "runtime"
sys.path.insert(0, str(RUNTIME))

from route_runtime import SemanticRouteRuntime  # noqa: E402
from static_obstacles import sample_static_obstacle_points, static_polyline_is_clear  # noqa: E402


def straight(start, goal):
    return [start, goal]


def test_semantic_route_advances_and_replan_replaces_local_corners():
    route = SemanticRouteRuntime(((1.0, 0.0, 0.0), (2.0, 0.0, 0.0)))
    assert route.initialize((0.0, 0.0, 0.0), straight)
    assert route.goals_xy() == [[1.0, 0.0]]
    route.update((1.0, 0.0, 0.0), 1.0, straight)
    assert route.semantic_progress == 1
    assert route.semantic_goal == (2.0, 0.0, 0.0)

    def around_wall(start, goal):
        return [start, (1.0, 1.0, 0.0), goal]

    assert route.replan((1.0, 0.0, 0.0), around_wall)
    assert route.goals_xy() == [[1.0, 1.0], [2.0, 0.0]]
    assert route.replan_count == 1


def test_prevalidated_cyclic_route_is_kept_as_local_authority():
    route = SemanticRouteRuntime(((1.0, 0.0, 0.0), (2.0, 0.0, 0.0)))
    assert route.initialize_resolved(
        (0.0, 0.0, 0.0),
        ((0.0, 0.0, 0.0), (0.5, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)),
    )
    assert route.goals_xy()[0] == [0.5, 0.0]
    route.update((1.0, 0.0, 0.0), 1.0, straight)
    assert route.semantic_progress == 1


def test_failed_replan_stops_only_that_route_state():
    route = SemanticRouteRuntime(((1.0, 0.0, 0.0),))
    assert not route.replan((0.0, 0.0, 0.0), lambda _start, _goal: None)
    assert route.replan_failure_count == 1


def test_failed_replan_can_skip_one_dense_corner_without_changing_pose():
    route = SemanticRouteRuntime(((2.0, 0.0, 0.0),))
    assert route.initialize_resolved(
        (0.0, 0.0, 0.0),
        ((0.0, 0.0, 0.0), (0.2, 0.0, 0.0), (0.4, 0.0, 0.0), (2.0, 0.0, 0.0)),
    )
    assert not route.replan((0.0, 0.0, 0.0), lambda _start, _goal: None)
    assert route.skip_blocked_local_corner()
    assert route.goals_xy()[0] == [0.4, 0.0]
    assert route.skipped_blocked_corner_count == 1


class FakeQuery:
    def raycast_all(self, origin, direction, distance, callback):
        callback({"collision": "/World/PedestrianColliders/self", "distance": 0.01})
        callback({"collision": "/World/StaticWall", "distance": 1.0})
        return True


def test_static_obstacles_ignore_people_and_return_world_points():
    points = sample_static_obstacle_points(FakeQuery(), (2.0, 3.0), ray_count=4)
    assert len(points) == 4
    assert points[0] == [3.27, 3.0]


def test_static_obstacles_accept_isaac_51_hit_objects():
    class Hit:
        def __init__(self, collision, distance):
            self.collision = collision
            self.distance = distance

    class ObjectQuery:
        @staticmethod
        def raycast_all(_origin, _direction, _distance, callback):
            callback(Hit("/World/PedestrianColliders/self", 0.01))
            callback(Hit("/World/StaticWall", 1.0))
            return True

    points = sample_static_obstacle_points(ObjectQuery(), (2.0, 3.0), ray_count=4)
    assert len(points) == 4
    assert points[0] == [3.27, 3.0]


def test_runtime_route_clearance_ignores_dynamic_actor_colliders():
    assert static_polyline_is_clear(
        FakeQuery(), ((2.0, 3.0, 0.0), (2.2, 3.0, 0.0))
    )
