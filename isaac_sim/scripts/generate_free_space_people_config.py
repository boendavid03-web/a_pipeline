#!/usr/bin/env python3
"""Build Gazebo-compatible IRA patrols in confirmed ROS-map free space.

SLAM maps encode three states: free, occupied, and unknown.  For character
navigation, unknown must be treated as blocked: a scan has not established
that a person can safely occupy it.  The generated patrol is an A* route on
the free cells after inflating every non-free cell by the requested pedestrian
clearance.  When a Gazebo scenario XML is supplied, its non-robot agent
clusters remain authoritative for route topology and population weights.  The
requested total is allocated with the same largest-remainder rule as
``scenario_pedestrian_controller.py`` and each person receives a deterministic
Gaussian speed sampled from the same seed.
"""

from __future__ import annotations

import argparse
import heapq
import math
import os
import random
import tempfile
import xml.etree.ElementTree as ET
from collections import deque
from copy import deepcopy
from pathlib import Path
from typing import Collection

import yaml

from people_route_geometry import (
    DEFAULT_LOBBY_BOUNDS,
    edge_is_continuously_safe,
    point_within_clear_bounds,
)


MAX_PEDESTRIAN_COUNT = 50
DEFAULT_SPAWN_CLEARANCE_M = 1.0
DEFAULT_MIN_PATROL_SEGMENT_M = 0.5
DEFAULT_MAX_PATROL_SEGMENT_M = 1.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-yaml", required=True, type=Path)
    parser.add_argument("--template", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--clearance", type=float, default=0.55)
    parser.add_argument(
        "--spawn-clearance",
        type=float,
        default=DEFAULT_SPAWN_CLEARANCE_M,
        help=(
            "Minimum deterministic centre-to-centre distance between generated "
            "pedestrian starts.  IRA uses a 0.5 m navigation radius, so the "
            "default prevents two characters from starting in the same social "
            "space."
        ),
    )
    parser.add_argument(
        "--min-patrol-segment",
        type=float,
        default=DEFAULT_MIN_PATROL_SEGMENT_M,
        help=(
            "Preferred minimum distance between intermediate patrol points. "
            "Shorter points are retained only when the inflated free-space "
            "geometry requires them to turn around an obstacle."
        ),
    )
    parser.add_argument(
        "--max-patrol-segment",
        type=float,
        default=DEFAULT_MAX_PATROL_SEGMENT_M,
        help="Maximum authored patrol edge length in metres.",
    )
    parser.add_argument(
        "--scenario",
        type=Path,
        help="Gazebo scenario XML supplying the six route clusters.",
    )
    parser.add_argument(
        "--pedestrian-count",
        type=int,
        default=-1,
        help="-1 keeps XML counts; otherwise allocate this total proportionally.",
    )
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument(
        "--world",
        type=Path,
        help="Optional Gazebo world; its static collision boxes are excluded too.",
    )
    return parser.parse_args()


def read_pgm(path: Path) -> tuple[int, int, int, bytes]:
    data = path.read_bytes()
    if not data.startswith(b"P5"):
        raise ValueError(f"{path} is not a binary PGM (P5) map")
    position = 2
    tokens: list[bytes] = []
    while len(tokens) < 3:
        while position < len(data) and data[position] in b" \t\r\n":
            position += 1
        if position < len(data) and data[position] == ord("#"):
            while position < len(data) and data[position] not in b"\r\n":
                position += 1
            continue
        end = position
        while end < len(data) and data[end] not in b" \t\r\n":
            end += 1
        tokens.append(data[position:end])
        position = end
    while position < len(data) and data[position] in b" \t\r\n":
        position += 1
    width, height, maximum = (int(token) for token in tokens)
    image = data[position:]
    if maximum > 255 or len(image) != width * height:
        raise ValueError(f"{path} has an unsupported or truncated PGM payload")
    return width, height, maximum, image


class FreeSpaceMap:
    def __init__(
        self,
        yaml_path: Path,
        clearance: float,
        static_boxes=(),
        bounds: tuple[float, float, float, float] = DEFAULT_LOBBY_BOUNDS,
    ) -> None:
        metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        if not isinstance(metadata, dict):
            raise ValueError(f"{yaml_path} does not contain map metadata")
        image_path = yaml_path.parent / str(metadata["image"])
        self.width, self.height, maximum, image = read_pgm(image_path)
        if maximum != 255:
            raise ValueError(f"{image_path} must use an 8-bit PGM payload")
        self.resolution = float(metadata["resolution"])
        origin = metadata["origin"]
        self.origin_x, self.origin_y = float(origin[0]), float(origin[1])
        free_threshold = float(metadata.get("free_thresh", 0.25))
        # ROS map_server reserves 205 for unknown in trinary maps.  It must not
        # be admitted merely because its grey value is numerically high.
        free_minimum = 255.0 * (1.0 - free_threshold)
        raw_free = {
            index
            for index, value in enumerate(image)
            if value != 205 and value >= free_minimum
        }
        if not raw_free:
            raise ValueError(f"{image_path} contains no confirmed-free cells")
        static_boxes = tuple(static_boxes)
        self.static_boxes = static_boxes
        self.bounds = tuple(bounds)
        self.clearance = clearance
        self._segment_cache: dict[tuple[int, int], bool] = {}
        # Inflate the occupancy image below, then apply exact oriented-box
        # clearance once.  Mixing the boxes into ``raw_free`` before erosion
        # applies the requested margin twice around those boxes.
        radius = math.ceil(clearance / self.resolution)
        disk = [
            (dx, dy)
            for dx in range(-radius, radius + 1)
            for dy in range(-radius, radius + 1)
            if math.hypot(dx, dy) * self.resolution <= clearance + 1.0e-9
        ]
        map_clear: set[int] = set()
        for cell in raw_free:
            x, y = cell % self.width, cell // self.width
            if all(
                0 <= x + dx < self.width
                and 0 <= y + dy < self.height
                and (y + dy) * self.width + x + dx in raw_free
                for dx, dy in disk
            ):
                map_clear.add(cell)
        if static_boxes:
            def avoids_static_box(cell: int) -> bool:
                x = self.origin_x + (cell % self.width + 0.5) * self.resolution
                y = self.origin_y + (
                    self.height - cell // self.width - 0.5
                ) * self.resolution
                for box in static_boxes:
                    if not edge_is_continuously_safe(
                        (x, y), (x, y), (box,), self.bounds, clearance
                    ):
                        return False
                return True

            self.free = {
                cell
                for cell in map_clear
                if point_within_clear_bounds(
                    (
                        self.origin_x + (cell % self.width + 0.5) * self.resolution,
                        self.origin_y
                        + (self.height - cell // self.width - 0.5) * self.resolution,
                    ),
                    self.bounds,
                    clearance,
                )
                and avoids_static_box(cell)
            }
        else:
            self.free = {
                cell
                for cell in map_clear
                if point_within_clear_bounds(
                    (
                        self.origin_x + (cell % self.width + 0.5) * self.resolution,
                        self.origin_y
                        + (self.height - cell // self.width - 0.5) * self.resolution,
                    ),
                    self.bounds,
                    clearance,
                )
            }
        if not self.free:
            raise ValueError(
                f"{image_path} has no free cells after {clearance:.2f} m inflation"
            )

    def xy(self, cell: int) -> tuple[int, int]:
        return cell % self.width, cell // self.width

    def world(self, cell: int) -> list[float]:
        x, raster_y = self.xy(cell)
        return [
            round(self.origin_x + (x + 0.5) * self.resolution, 4),
            round(self.origin_y + (self.height - raster_y - 0.5) * self.resolution, 4),
            0.0,
        ]

    def cell_for_world(self, x: float, y: float) -> int | None:
        column = math.floor((x - self.origin_x) / self.resolution)
        row_from_bottom = math.floor((y - self.origin_y) / self.resolution)
        row = self.height - 1 - row_from_bottom
        if not (0 <= column < self.width and 0 <= row < self.height):
            return None
        return row * self.width + column

    def contains_world(self, x: float, y: float) -> bool:
        cell = self.cell_for_world(x, y)
        return cell in self.free if cell is not None else False

    def nearest(
        self,
        x: float,
        y: float,
        candidates: Collection[int] | None = None,
    ) -> int:
        """Return the nearest confirmed-free cell, optionally in one component."""
        cells = self.free if candidates is None else candidates
        if not cells:
            raise ValueError("cannot select a nearest cell from an empty set")
        requested = self.cell_for_world(x, y)
        if requested in cells:
            return requested
        return min(
            cells,
            key=lambda cell: (
                (self.world(cell)[0] - x) ** 2 + (self.world(cell)[1] - y) ** 2,
                cell,
            ),
        )

    def nearest_separated(
        self,
        x: float,
        y: float,
        occupied: Collection[int],
        minimum_separation: float,
        candidates: Collection[int] | None = None,
    ) -> int:
        """Return the nearest free cell outside every occupied social radius."""
        cells = self.free if candidates is None else candidates
        occupied_cells = tuple(occupied)
        if not occupied_cells or minimum_separation <= 0.0:
            return self.nearest(x, y, candidates=cells)
        minimum_squared = minimum_separation * minimum_separation
        occupied_world = [self.world(cell) for cell in occupied_cells]
        separated = [
            cell
            for cell in cells
            if all(
                (self.world(cell)[0] - other[0]) ** 2
                + (self.world(cell)[1] - other[1]) ** 2
                >= minimum_squared - 1.0e-9
                for other in occupied_world
            )
        ]
        if not separated:
            raise ValueError(
                "cannot place another pedestrian with "
                f"{minimum_separation:.2f} m start separation"
            )
        return self.nearest(x, y, candidates=separated)

    def _segment_raster_free(
        self, start_x: float, start_y: float, end_x: float, end_y: float
    ) -> bool:
        steps = max(
            1,
            math.ceil(
                math.hypot(end_x - start_x, end_y - start_y)
                / (self.resolution * 0.25)
            ),
        )
        return all(
            self.contains_world(
                start_x + (end_x - start_x) * index / steps,
                start_y + (end_y - start_y) * index / steps,
            )
            for index in range(steps + 1)
        )

    def segment_world_free(
        self, start_x: float, start_y: float, end_x: float, end_y: float
    ) -> bool:
        return self._segment_raster_free(
            start_x, start_y, end_x, end_y
        ) and edge_is_continuously_safe(
            (start_x, start_y),
            (end_x, end_y),
            self.static_boxes,
            self.bounds,
            self.clearance,
        )

    def components(self) -> list[list[int]]:
        remaining = set(self.free)
        groups: list[list[int]] = []
        while remaining:
            root = remaining.pop()
            queue = deque([root])
            group = [root]
            while queue:
                current = queue.popleft()
                x, y = self.xy(current)
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nx, ny = x + dx, y + dy
                    neighbour = ny * self.width + nx
                    if 0 <= nx < self.width and 0 <= ny < self.height and neighbour in remaining:
                        remaining.remove(neighbour)
                        queue.append(neighbour)
                        group.append(neighbour)
            groups.append(group)
        return groups

    def route(self, start: int, goal: int) -> list[int]:
        def estimate(cell: int) -> float:
            x, y = self.xy(cell)
            gx, gy = self.xy(goal)
            return math.hypot(x - gx, y - gy)

        heap: list[tuple[float, float, int]] = [(estimate(start), 0.0, start)]
        previous: dict[int, int] = {}
        cost = {start: 0.0}
        while heap:
            _, current_cost, current = heapq.heappop(heap)
            if current == goal:
                break
            if current_cost != cost.get(current):
                continue
            x, y = self.xy(current)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = x + dx, y + dy
                neighbour = ny * self.width + nx
                if not (0 <= nx < self.width and 0 <= ny < self.height) or neighbour not in self.free:
                    continue
                if not self.segment_free(current, neighbour):
                    continue
                proposed = current_cost + 1.0
                if proposed < cost.get(neighbour, math.inf):
                    cost[neighbour] = proposed
                    previous[neighbour] = current
                    heapq.heappush(heap, (proposed + estimate(neighbour), proposed, neighbour))
        if goal not in cost:
            raise ValueError("selected SLAM free cells are not connected")
        path = [goal]
        while path[-1] != start:
            path.append(previous[path[-1]])
        path.reverse()
        return path

    def segment_free(self, start: int, end: int) -> bool:
        cache_key = (min(start, end), max(start, end))
        cached = self._segment_cache.get(cache_key)
        if cached is not None:
            return cached
        start_world = self.world(start)
        end_world = self.world(end)
        result = self.segment_world_free(
            start_world[0], start_world[1], end_world[0], end_world[1]
        )
        self._segment_cache[cache_key] = result
        return result

    def simplify(
        self,
        path: list[int],
        minimum_segment: float = DEFAULT_MIN_PATROL_SEGMENT_M,
        maximum_segment: float = DEFAULT_MAX_PATROL_SEGMENT_M,
    ) -> list[int]:
        """String-pull an A* path without creating stop-and-go micro-segments.

        A purely greedy string pull takes the farthest visible cell at every
        step.  Near an inflated obstacle corner that can leave only one or two
        grid cells for the following edge.  IRA's BehaviorAgent treats every
        authored point as a steering target, so those 5--15 cm tails cause an
        unnecessary brake/accelerate cycle.

        Keep the greedy path as a guaranteed-safe backbone, add deterministic
        candidate cells along the A* path, then solve a forward DAG.  Its
        lexicographic objective first minimizes sub-threshold edges, then the
        number of steering points, and finally maximizes the shortest edge.
        Every shortcut is still checked against the inflated free-space map;
        consequently a genuinely necessary obstacle corner is never removed.
        """
        if not path:
            raise ValueError("cannot simplify an empty path")
        if len(path) == 1:
            return list(path)
        if not math.isfinite(minimum_segment) or minimum_segment <= 0.0:
            raise ValueError("minimum_segment must be a positive finite number")
        if not math.isfinite(maximum_segment) or maximum_segment <= 0.0:
            raise ValueError("maximum_segment must be a positive finite number")

        greedy_indices = [0]
        index = 0
        while index < len(path) - 1:
            candidate = len(path) - 1
            while (
                candidate > index + 1
                and not self.segment_free(path[index], path[candidate])
            ):
                candidate -= 1
            greedy_indices.append(candidate)
            index = candidate

        # Sampling the entire raster path would make the visibility graph
        # quadratic in paths that cross the lobby.  Half-threshold
        # neighbourhoods around each greedy turn preserve precise corner
        # choices, while the threshold-spaced backbone supplies alternatives
        # on long straight sections.  All original greedy nodes are retained,
        # so this candidate graph always contains a safe start-to-goal path.
        stride = max(1, int(math.floor(minimum_segment / self.resolution)))
        corner_radius = max(1, int(math.ceil(0.5 * minimum_segment / self.resolution)))
        candidate_indices = set(range(0, len(path), stride))
        candidate_indices.update((0, len(path) - 1))
        for turn in greedy_indices:
            candidate_indices.update(
                range(
                    max(0, turn - corner_radius),
                    min(len(path), turn + corner_radius + 1),
                )
            )
        candidates = sorted(candidate_indices)
        candidate_world = [self.world(path[path_index]) for path_index in candidates]

        # (short edge count, steering edge count, negative bottleneck length).
        # The bottleneck tie-break avoids choosing a 7 cm edge when two paths
        # have the same unavoidable number of short obstacle-corner edges.
        scores: list[tuple[int, int, float] | None] = [None] * len(candidates)
        predecessors: list[int | None] = [None] * len(candidates)
        bottlenecks = [0.0] * len(candidates)
        scores[0] = (0, 0, -math.inf)
        bottlenecks[0] = math.inf
        visibility: dict[tuple[int, int], bool] = {}

        for current in range(1, len(candidates)):
            for previous in range(current):
                previous_score = scores[previous]
                if previous_score is None:
                    continue
                visibility_key = (previous, current)
                is_visible = visibility.get(visibility_key)
                if is_visible is None:
                    is_visible = self.segment_free(
                        path[candidates[previous]], path[candidates[current]]
                    )
                    visibility[visibility_key] = is_visible
                if not is_visible:
                    continue
                distance = math.dist(
                    candidate_world[previous][:2], candidate_world[current][:2]
                )
                bottleneck = min(bottlenecks[previous], distance)
                score = (
                    previous_score[0]
                    + int(distance < minimum_segment - 1.0e-9),
                    previous_score[1] + 1,
                    -bottleneck,
                )
                if scores[current] is None or score < scores[current]:
                    scores[current] = score
                    predecessors[current] = previous
                    bottlenecks[current] = bottleneck

        if scores[-1] is None:
            # This should be impossible because the candidate set contains the
            # complete greedy backbone, but make a safety failure explicit.
            raise ValueError("patrol simplification lost its free-space path")
        selected: list[int] = []
        current: int | None = len(candidates) - 1
        while current is not None:
            selected.append(path[candidates[current]])
            current = predecessors[current]
        selected.reverse()
        # Keep long shortcuts bounded using only the original safe A* cells;
        # never interpolate a point through an obstacle.
        path_indices = {cell: index for index, cell in enumerate(path)}
        bounded: list[int] = [selected[0]]
        for start, end in zip(selected, selected[1:]):
            start_index = path_indices[start]
            end_index = path_indices[end]
            cursor = start_index
            while cursor < end_index:
                candidate = end_index
                while candidate > cursor + 1:
                    distance = math.dist(
                        self.world(path[cursor])[:2], self.world(path[candidate])[:2]
                    )
                    if distance <= maximum_segment + 1.0e-9 and self.segment_free(
                        path[cursor], path[candidate]
                    ):
                        break
                    candidate -= 1
                bounded.append(path[candidate])
                cursor = candidate
        return bounded

    def merge_short_visible(
        self,
        path: list[int],
        minimum_segment: float = DEFAULT_MIN_PATROL_SEGMENT_M,
        maximum_segment: float = DEFAULT_MAX_PATROL_SEGMENT_M,
    ) -> list[int]:
        """Remove short join points only when their neighbours have clear LOS.

        ``routed_loop`` simplifies each authored leg separately so the Gazebo
        route topology remains authoritative.  A projected route anchor can,
        however, be nearly collinear with the last point of one leg and the
        first point of the next.  This final pass merges such joins while
        preserving the explicit first/last spawn and every obstacle corner
        whose neighbours cannot be connected safely.
        """
        result = list(path)
        if not math.isfinite(maximum_segment) or maximum_segment <= 0.0:
            raise ValueError("maximum_segment must be a positive finite number")
        while len(result) > 2:
            removable: list[tuple[float, int]] = []
            for index in range(1, len(result) - 1):
                before = self.world(result[index - 1])
                current = self.world(result[index])
                after = self.world(result[index + 1])
                shorter_adjacent = min(
                    math.dist(before[:2], current[:2]),
                    math.dist(current[:2], after[:2]),
                )
                if (
                    shorter_adjacent < minimum_segment - 1.0e-9
                    and math.dist(before[:2], after[:2])
                    <= maximum_segment + 1.0e-9
                    and self.segment_free(result[index - 1], result[index + 1])
                ):
                    removable.append((shorter_adjacent, index))
            if not removable:
                break
            _, remove_index = min(removable)
            result.pop(remove_index)
        return result


def farthest_points(grid: FreeSpaceMap, component: list[int]) -> list[int]:
    """Choose four spread-out anchors, deterministically, in one free component."""
    # Bounding-box extremes avoid a random route changing whenever YAML parser
    # ordering changes, while still adapting to the actually mapped free area.
    ordered = sorted(component, key=lambda cell: (grid.xy(cell)[0], grid.xy(cell)[1]))
    first = ordered[0]
    selected = [first]
    while len(selected) < 4:
        candidate = max(
            component,
            key=lambda cell: min(
                (grid.xy(cell)[0] - grid.xy(other)[0]) ** 2
                + (grid.xy(cell)[1] - grid.xy(other)[1]) ** 2
                for other in selected
            ),
        )
        if candidate in selected:
            break
        selected.append(candidate)
    return selected


def make_loop(
    grid: FreeSpaceMap,
    minimum_segment: float = DEFAULT_MIN_PATROL_SEGMENT_M,
    maximum_segment: float = DEFAULT_MAX_PATROL_SEGMENT_M,
) -> list[list[float]]:
    component = max(grid.components(), key=len)
    if len(component) < 2:
        raise ValueError("largest inflated free-space component is too small for a patrol")
    anchors = farthest_points(grid, component)
    # Simplify each A* leg independently.  Simplifying the concatenated closed
    # loop would see its final copy of the start as a zero-length shortcut and
    # collapse a valid patrol into one stationary point.
    cells: list[int] = []
    for start, end in zip(anchors, anchors[1:] + anchors[:1]):
        route = grid.simplify(
            grid.route(start, end), minimum_segment, maximum_segment
        )
        cells.extend(route if not cells else route[1:])
    cells = grid.merge_short_visible(cells, minimum_segment, maximum_segment)
    # The final point deliberately duplicates the start.  IRA closes patrols,
    # so this makes the closing edge zero-length rather than an unsafe chord.
    if cells[-1] != cells[0] or len(cells) < 3:
        raise ValueError("generated patrol is not a valid closed free-space loop")
    if not all(
        grid.segment_free(start, end)
        and math.dist(grid.world(start)[:2], grid.world(end)[:2])
        <= maximum_segment + 1.0e-9
        for start, end in zip(cells, cells[1:])
    ):
        raise ValueError("generated patrol contains a segment outside free space")
    if not (
        grid.segment_free(cells[-1], cells[0])
        and math.dist(grid.world(cells[-1])[:2], grid.world(cells[0])[:2])
        <= maximum_segment + 1.0e-9
    ):
        raise ValueError("generated patrol closing segment leaves confirmed free space")
    return [grid.world(cell) for cell in cells]


def allocate_pedestrian_counts(configured: list[int], requested: int) -> list[int]:
    """Match Gazebo's proportional largest-remainder population allocation."""
    if requested < -1:
        raise ValueError("pedestrian_count must be -1 or a non-negative integer")
    if requested == -1:
        return list(configured)
    if requested == 0:
        return [0] * len(configured)
    total = sum(configured)
    if total <= 0:
        raise ValueError("requested pedestrians but the scenario has no clusters")
    scaled = [requested * count / total for count in configured]
    allocated = [int(value) for value in scaled]
    remainder = requested - sum(allocated)
    priority = sorted(
        range(len(configured)),
        key=lambda index: (
            scaled[index] - allocated[index],
            configured[index],
            -index,
        ),
        reverse=True,
    )
    for index in priority[:remainder]:
        allocated[index] += 1
    return allocated


def load_gazebo_clusters(path: Path) -> list[dict[str, object]]:
    """Read non-robot clusters and resolve their waypoint coordinates."""
    root = ET.parse(path).getroot()
    waypoints = {
        element.attrib["id"]: (
            float(element.attrib["x"]),
            float(element.attrib["y"]),
        )
        for element in root
        if element.tag == "waypoint"
    }
    clusters: list[dict[str, object]] = []
    for element in root:
        if element.tag != "agent" or int(element.attrib.get("type", "0")) == 2:
            continue
        route = [
            waypoints[child.attrib["id"]]
            for child in element
            if child.tag == "addwaypoint" and child.attrib["id"] in waypoints
        ]
        if not route:
            continue
        clusters.append(
            {
                "count": max(0, int(element.attrib.get("n", "1"))),
                "center": (float(element.attrib["x"]), float(element.attrib["y"])),
                "extent": (
                    float(element.attrib.get("dx", "0")),
                    float(element.attrib.get("dy", "0")),
                ),
                "route": route,
            }
        )
    if not clusters:
        raise ValueError(f"{path} contains no non-robot waypoint clusters")
    return clusters


def routed_loop(
    grid: FreeSpaceMap,
    start: tuple[float, float],
    route: list[tuple[float, float]],
    component: Collection[int] | None = None,
    minimum_segment: float = DEFAULT_MIN_PATROL_SEGMENT_M,
    leg_cache: dict[tuple[int, int, float, float], tuple[int, ...]] | None = None,
    maximum_segment: float = DEFAULT_MAX_PATROL_SEGMENT_M,
) -> list[list[float]]:
    """Route one initial spawn and a cyclic Gazebo waypoint sequence on A*."""
    anchors = [
        grid.nearest(*start, candidates=component),
        *(grid.nearest(*point, candidates=component) for point in route),
    ]
    # Initial spawn -> route[0], then every cyclic leg including last -> first.
    route_cells = anchors[1:]
    legs = [(anchors[0], route_cells[0])]
    legs.extend(zip(route_cells, route_cells[1:] + route_cells[:1]))
    # IRA closes the authored path from its last point to its first.  Return
    # explicitly to the sampled spawn so that implicit closing edge has zero
    # length instead of cutting across a wall.
    legs.append((anchors[1], anchors[0]))
    cells: list[int] = []
    for leg_start, leg_end in legs:
        cache_key = (leg_start, leg_end, minimum_segment, maximum_segment)
        cached_leg = leg_cache.get(cache_key) if leg_cache is not None else None
        if cached_leg is None:
            leg = grid.simplify(
                grid.route(leg_start, leg_end), minimum_segment, maximum_segment
            )
            if leg_cache is not None:
                leg_cache[cache_key] = tuple(leg)
        else:
            leg = list(cached_leg)
        cells.extend(leg if not cells else leg[1:])
    if len(cells) < 2:
        raise ValueError("generated Gazebo patrol has fewer than two points")
    cells = grid.merge_short_visible(cells, minimum_segment, maximum_segment)
    if not all(
        grid.segment_free(start, end)
        and math.dist(grid.world(start)[:2], grid.world(end)[:2])
        <= maximum_segment + 1.0e-9
        for start, end in zip(cells, cells[1:] + cells[:1])
    ):
        raise ValueError("generated Gazebo patrol contains an unsafe edge")
    return [grid.world(cell) for cell in cells]


def validate_generated_routes(
    groups: dict[str, dict[str, object]],
    grid: FreeSpaceMap,
    maximum_segment: float = DEFAULT_MAX_PATROL_SEGMENT_M,
) -> None:
    """Validate every authored edge, including the patrol closing edge."""
    for group_name, group in groups.items():
        for routine in group.get("routines", []):
            patrol = routine.get("patrol")
            if patrol is None:
                continue
            points = patrol.get("path_points", [])
            if len(points) < 2:
                raise ValueError(f"{group_name}: patrol needs at least two points")
            planar = [(float(point[0]), float(point[1])) for point in points]
            for index, (start, end) in enumerate(
                zip(planar, planar[1:] + planar[:1])
            ):
                if not grid.segment_world_free(*start, *end):
                    raise ValueError(
                        f"{group_name}: generated patrol edge {index} is unsafe"
                    )
                if math.dist(start, end) > maximum_segment + 1.0e-9:
                    raise ValueError(
                        f"{group_name}: generated patrol edge {index} exceeds "
                        f"{maximum_segment:.3f} m"
                    )


def gazebo_compatible_groups(
    template_groups: dict[str, dict[str, object]],
    clusters: list[dict[str, object]],
    counts: list[int],
    grid: FreeSpaceMap,
    rng: random.Random,
    base_speed: float,
    component: Collection[int] | None = None,
    spawn_clearance: float = DEFAULT_SPAWN_CLEARANCE_M,
    minimum_segment: float = DEFAULT_MIN_PATROL_SEGMENT_M,
    maximum_segment: float = DEFAULT_MAX_PATROL_SEGMENT_M,
) -> dict[str, dict[str, object]]:
    """Expand to one IRA group per person so vmax is deterministic per agent."""
    if len(template_groups) != len(clusters):
        raise ValueError(
            "template group count must match Gazebo pedestrian cluster count: "
            f"template={len(template_groups)} scenario={len(clusters)}"
        )
    if not math.isfinite(spawn_clearance) or spawn_clearance <= 0.0:
        raise ValueError("spawn_clearance must be a positive finite number")
    if not math.isfinite(minimum_segment) or minimum_segment <= 0.0:
        raise ValueError("minimum_segment must be a positive finite number")
    if not math.isfinite(maximum_segment) or maximum_segment <= 0.0:
        raise ValueError("maximum_segment must be a positive finite number")
    generated: dict[str, dict[str, object]] = {}
    occupied_start_cells: list[int] = []
    leg_cache: dict[tuple[int, int, float, float], tuple[int, ...]] = {}
    for cluster_index, ((template_name, template), cluster, count) in enumerate(
        zip(template_groups.items(), clusters, counts)
    ):
        center_x, center_y = cluster["center"]
        extent_x, extent_y = cluster["extent"]
        route = cluster["route"]
        for person_index in range(count):
            requested_start = (
                center_x + rng.uniform(-extent_x / 2.0, extent_x / 2.0),
                center_y + rng.uniform(-extent_y / 2.0, extent_y / 2.0),
            )
            start_cell = grid.nearest_separated(
                *requested_start,
                occupied_start_cells,
                spawn_clearance,
                candidates=component,
            )
            occupied_start_cells.append(start_cell)
            start_world = grid.world(start_cell)
            start = (float(start_world[0]), float(start_world[1]))
            speed = max(0.1, rng.gauss(base_speed, 0.26))
            # Gazebo's social-force controller can safely let a whole cluster
            # converge on the same first waypoint.  Isaac BehaviorAgent's
            # reciprocal avoidance is weaker for same-direction followers,
            # so phase each person around the *same cyclic route* instead of
            # creating an artificial queue at route[0].  Route topology and
            # direction are unchanged; only the cyclic entry phase differs.
            route_phase = person_index % len(route)
            phased_route = route[route_phase:] + route[:route_phase]
            group = deepcopy(template)
            group["num"] = 1
            patrol_found = False
            for routine in group.get("routines", []):
                patrol = routine.get("patrol")
                if patrol is None:
                    continue
                patrol_found = True
                patrol["speed_range"] = [round(speed, 6), round(speed, 6)]
                patrol["path_points"] = routed_loop(
                    grid,
                    start,
                    phased_route,
                    component,
                    minimum_segment,
                    leg_cache,
                    maximum_segment,
                )
            if not patrol_found:
                raise ValueError(f"template group {template_name!r} has no patrol")
            name = f"gazebo_{chr(ord('a') + cluster_index)}_{person_index + 1:03d}"
            generated[name] = group
    return generated


def main() -> int:
    args = parse_args()
    if not math.isfinite(args.clearance) or args.clearance <= 0.0:
        raise SystemExit("ERROR: --clearance must be a positive finite number")
    if not math.isfinite(args.spawn_clearance) or args.spawn_clearance <= 0.0:
        raise SystemExit(
            "ERROR: --spawn-clearance must be a positive finite number"
        )
    if not math.isfinite(args.min_patrol_segment) or args.min_patrol_segment <= 0.0:
        raise SystemExit(
            "ERROR: --min-patrol-segment must be a positive finite number"
        )
    if (
        not math.isfinite(args.max_patrol_segment)
        or args.max_patrol_segment <= 0.0
    ):
        raise SystemExit(
            "ERROR: --max-patrol-segment must be a positive finite number"
        )
    if not -1 <= args.pedestrian_count <= MAX_PEDESTRIAN_COUNT:
        raise SystemExit(
            "ERROR: --pedestrian-count must be -1 or between 0 and "
            f"{MAX_PEDESTRIAN_COUNT}"
        )
    if not 0 <= args.seed <= 4_294_967_295:
        raise SystemExit("ERROR: --seed must be between 0 and 4294967295")
    if not math.isfinite(args.speed) or args.speed <= 0.0:
        raise SystemExit("ERROR: --speed must be a positive finite number")
    static_boxes = ()
    if args.world is not None:
        from convert_gazebo_boxes_to_usda import load_static_boxes

        static_boxes, _ = load_static_boxes(args.world)
    grid = FreeSpaceMap(
        args.map_yaml,
        args.clearance,
        static_boxes,
        bounds=DEFAULT_LOBBY_BOUNDS,
    )
    config = yaml.safe_load(args.template.read_text(encoding="utf-8"))
    groups = config["isaacsim.replicator.agent"]["character"]["groups"]
    allocation = None
    if args.scenario is not None:
        clusters = load_gazebo_clusters(args.scenario)
        patrol_component = max(grid.components(), key=len)
        allocation = allocate_pedestrian_counts(
            [int(cluster["count"]) for cluster in clusters],
            args.pedestrian_count,
        )
        config["isaacsim.replicator.agent"]["seed"] = args.seed
        groups = gazebo_compatible_groups(
            groups,
            clusters,
            allocation,
            grid,
            random.Random(args.seed),
            args.speed,
            patrol_component,
            args.spawn_clearance,
            args.min_patrol_segment,
            args.max_patrol_segment,
        )
        config["isaacsim.replicator.agent"]["character"]["groups"] = groups
        point_count = sum(
            len(routine["patrol"]["path_points"])
            for group in groups.values()
            for routine in group.get("routines", [])
            if "patrol" in routine
        )
    else:
        loop = make_loop(grid, args.min_patrol_segment, args.max_patrol_segment)
        point_count = len(loop)
        for offset, group in enumerate(groups.values()):
            points = loop[offset:] + loop[:offset]
            for routine in group.get("routines", []):
                if "patrol" in routine:
                    routine["patrol"]["path_points"] = [list(point) for point in points]
    validate_generated_routes(groups, grid, args.max_patrol_segment)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=args.output.parent,
            prefix=f".{args.output.name}.",
            suffix=".tmp",
            delete=False,
        ) as handle:
            temporary_path = handle.name
            yaml.safe_dump(config, handle, sort_keys=False)
        os.replace(temporary_path, args.output)
        temporary_path = None
    finally:
        if temporary_path is not None:
            try:
                os.unlink(temporary_path)
            except FileNotFoundError:
                pass
    print(
        "SLAM_FREE_SPACE_PATROL=PASS "
        f"map={args.map_yaml} free_cells={len(grid.free)} static_boxes={len(static_boxes)} "
        f"people={sum(allocation) if allocation is not None else sum(int(group['num']) for group in groups.values())} "
        f"allocation={allocation} seed={args.seed} speed={args.speed:.3f} "
        f"points={point_count} clearance_m={args.clearance:.2f} "
        f"spawn_clearance_m={args.spawn_clearance:.2f} output={args.output}"
        f" min_patrol_segment_m={args.min_patrol_segment:.2f}"
        f" max_patrol_segment_m={args.max_patrol_segment:.2f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
