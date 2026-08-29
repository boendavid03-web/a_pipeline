#!/usr/bin/env python3
"""Fit and validate the engineering-lobby occupancy-map to world transform.

The fitted transform has TF direction ``map -> odom`` and maps coordinates as

    p_map = R(yaw) * p_odom + [x, y]

The current custom-scene bridge publishes odom coordinates directly from the
Z-up Isaac/Gazebo world, so no Isaac runtime is needed for this calculation.
The source occupancy map and world are never modified.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import re
import sys
import warnings
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

# Ubuntu 22.04's SciPy emits a compatibility warning when another workspace has
# upgraded NumPy.  The two routines used here are covered by this script's
# deterministic repeatability check, so keep the machine-readable output clean.
warnings.filterwarnings("ignore", category=UserWarning, module="scipy")
from scipy.ndimage import distance_transform_edt, map_coordinates  # noqa: E402
from scipy.optimize import differential_evolution  # noqa: E402


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[2]
DEFAULT_MAP_YAML = (
    PROJECT_ROOT
    / "workspaces/ros2_ws/src/semantic_nav_gazebo/maps/gazebo_eng_lobby"
    / "gazebo_eng_lobby.yaml"
)
DEFAULT_WORLD = (
    PROJECT_ROOT
    / "workspaces/ros2_ws/src/semantic_nav_gazebo/worlds/gazebo_eng_lobby.world"
)
DEFAULT_USDA = PROJECT_ROOT / "isaac_sim/scenes/a_pipeline_eng_lobby.usda"
CONVERTER = PROJECT_ROOT / "isaac_sim/scripts/convert_gazebo_boxes_to_usda.py"


@dataclass(frozen=True)
class ResidualStats:
    sample_count: int
    median_m: float
    p90_m: float
    p95_m: float
    maximum_m: float
    within_0_15_fraction: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-yaml", type=Path, default=DEFAULT_MAP_YAML)
    parser.add_argument("--world", type=Path, default=DEFAULT_WORLD)
    parser.add_argument("--usda", type=Path, default=DEFAULT_USDA)
    parser.add_argument("--spawn-x", type=float, default=2.0)
    parser.add_argument("--spawn-y", type=float, default=2.0)
    parser.add_argument("--spawn-yaw", type=float, default=0.0)
    parser.add_argument("--samples-per-edge", type=int, default=40)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--overlay", type=Path)
    return parser.parse_args()


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_converter() -> Any:
    spec = importlib.util.spec_from_file_location("a_pipeline_scene_converter", CONVERTER)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import scene converter: {CONVERTER}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def read_pgm(path: Path) -> np.ndarray:
    data = path.read_bytes()
    offset = 0
    tokens: list[bytes] = []
    while len(tokens) < 4:
        newline = data.find(b"\n", offset)
        if newline < 0:
            raise ValueError(f"invalid PGM header: {path}")
        line = data[offset:newline].strip()
        offset = newline + 1
        if line and not line.startswith(b"#"):
            tokens.extend(line.split())
    if tokens[0] != b"P5":
        raise ValueError(f"expected binary P5 PGM: {path}")
    width, height, maximum = (int(value) for value in tokens[1:4])
    if maximum != 255:
        raise ValueError(f"expected 8-bit PGM, max={maximum}: {path}")
    pixels = np.frombuffer(data, dtype=np.uint8, count=width * height, offset=offset)
    if pixels.size != width * height:
        raise ValueError(f"truncated PGM payload: {path}")
    return pixels.reshape(height, width)


def sample_box_perimeters(boxes: list[Any], samples_per_edge: int) -> tuple[np.ndarray, np.ndarray]:
    if samples_per_edge < 8:
        raise ValueError("--samples-per-edge must be at least 8")
    points: list[tuple[float, float]] = []
    owners: list[int] = []
    edge_parameters = np.linspace(-0.5, 0.5, samples_per_edge, endpoint=False)
    for box_index, box in enumerate(boxes):
        center_x, center_y = box.pose.x, box.pose.y
        size_x, size_y = box.size[:2]
        cosine, sine = math.cos(box.pose.yaw), math.sin(box.pose.yaw)
        for value in edge_parameters:
            local_points = (
                (value * size_x, -0.5 * size_y),
                (0.5 * size_x, value * size_y),
                (-value * size_x, 0.5 * size_y),
                (-0.5 * size_x, -value * size_y),
            )
            for local_x, local_y in local_points:
                points.append(
                    (
                        center_x + cosine * local_x - sine * local_y,
                        center_y + sine * local_x + cosine * local_y,
                    )
                )
                owners.append(box_index)
    return np.asarray(points, dtype=float), np.asarray(owners, dtype=int)


def residual_stats(values: np.ndarray) -> ResidualStats:
    return ResidualStats(
        sample_count=int(values.size),
        median_m=float(np.median(values)),
        p90_m=float(np.percentile(values, 90)),
        p95_m=float(np.percentile(values, 95)),
        maximum_m=float(np.max(values)),
        within_0_15_fraction=float(np.mean(values <= 0.15)),
    )


def render_overlay(
    destination: Path,
    image: np.ndarray,
    origin: tuple[float, float, float],
    resolution: float,
    boxes: list[Any],
    transform: np.ndarray,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    tx, ty, yaw = (float(value) for value in transform)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    height, width = image.shape
    extent = [
        origin[0],
        origin[0] + width * resolution,
        origin[1],
        origin[1] + height * resolution,
    ]
    figure, axes = plt.subplots(1, 4, figsize=(20, 5), sharey=True)
    # The three detail panels use the same world-X box-count tertile boundaries
    # as the residual report (currently 11.25 m and 19.0 m).
    box_centers_x = np.asarray([box.pose.x for box in boxes], dtype=float)
    west_center, center_east = np.quantile(box_centers_x, [1.0 / 3.0, 2.0 / 3.0])
    regions = (
        ("whole", -5.0, 36.0),
        ("west", -5.0, float(west_center)),
        ("center", float(west_center), float(center_east)),
        ("east", float(center_east), 36.0),
    )
    for axis, (name, minimum_x, maximum_x) in zip(axes, regions):
        axis.imshow(image, cmap="gray", origin="upper", extent=extent, vmin=0, vmax=255)
        for box in boxes:
            size_x, size_y = box.size[:2]
            box_cosine, box_sine = math.cos(box.pose.yaw), math.sin(box.pose.yaw)
            local_corners = (
                (-0.5 * size_x, -0.5 * size_y),
                (0.5 * size_x, -0.5 * size_y),
                (0.5 * size_x, 0.5 * size_y),
                (-0.5 * size_x, 0.5 * size_y),
                (-0.5 * size_x, -0.5 * size_y),
            )
            map_corners = []
            for local_x, local_y in local_corners:
                world_x = box.pose.x + box_cosine * local_x - box_sine * local_y
                world_y = box.pose.y + box_sine * local_x + box_cosine * local_y
                map_corners.append(
                    (
                        cosine * world_x - sine * world_y + tx,
                        sine * world_x + cosine * world_y + ty,
                    )
                )
            axis.plot(
                [point[0] for point in map_corners],
                [point[1] for point in map_corners],
                color="red",
                linewidth=0.7,
                alpha=0.85,
            )
        axis.set_xlim(minimum_x, maximum_x)
        axis.set_ylim(-3.0, 26.0)
        axis.set_aspect("equal")
        axis.set_title(name)
        axis.grid(alpha=0.15)
    figure.suptitle("PGM occupancy (black) with transformed world boxes (red)")
    figure.tight_layout()
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=180)
    plt.close(figure)


def main() -> int:
    args = parse_args()
    map_yaml = args.map_yaml.expanduser().resolve()
    world = args.world.expanduser().resolve()
    usda = args.usda.expanduser().resolve()
    for path in (map_yaml, world, usda, CONVERTER):
        if not path.is_file():
            raise ValueError(f"required input does not exist: {path}")

    map_config = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    required_map_fields = {
        "image", "resolution", "origin", "negate", "occupied_thresh", "free_thresh"
    }
    if not isinstance(map_config, dict) or not required_map_fields <= map_config.keys():
        raise ValueError(f"incomplete occupancy-map YAML: {map_yaml}")
    image_path = (map_yaml.parent / str(map_config["image"])).resolve()
    image = read_pgm(image_path)
    resolution = float(map_config["resolution"])
    origin_values = tuple(float(value) for value in map_config["origin"])
    if len(origin_values) != 3 or resolution <= 0.0:
        raise ValueError("invalid map resolution/origin")
    occupied_probability = (255.0 - image.astype(float)) / 255.0
    if int(map_config["negate"]) != 0:
        occupied_probability = 1.0 - occupied_probability
    occupied = occupied_probability > float(map_config["occupied_thresh"])
    distance_field = distance_transform_edt(~occupied) * resolution

    converter = load_converter()
    boxes, skipped_includes = converter.load_static_boxes(world)
    if len(boxes) != 79:
        raise ValueError(f"expected 79 static boxes, found {len(boxes)}")
    usda_text = usda.read_text(encoding="utf-8")
    source_hash_match = re.search(r'sourceWorldSha256 = "([0-9a-f]{64})"', usda_text)
    box_count_match = re.search(r"migratedStaticBoxes = ([0-9]+)", usda_text)
    world_hash = sha256(world)
    if source_hash_match is None or source_hash_match.group(1) != world_hash:
        raise ValueError("USDA sourceWorldSha256 does not match the current Gazebo world")
    if box_count_match is None or int(box_count_match.group(1)) != len(boxes):
        raise ValueError("USDA migratedStaticBoxes does not match the current world")

    world_points, owners = sample_box_perimeters(boxes, args.samples_per_edge)
    height, width = image.shape
    origin_x, origin_y = origin_values[:2]

    def distances(transform: np.ndarray) -> np.ndarray:
        tx, ty, yaw = (float(value) for value in transform)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        map_x = cosine * world_points[:, 0] - sine * world_points[:, 1] + tx
        map_y = sine * world_points[:, 0] + cosine * world_points[:, 1] + ty
        columns = (map_x - origin_x) / resolution - 0.5
        rows = height - 0.5 - (map_y - origin_y) / resolution
        inside = (
            (columns >= 0.0)
            & (columns <= width - 1)
            & (rows >= 0.0)
            & (rows <= height - 1)
        )
        output = np.full(world_points.shape[0], 5.0, dtype=float)
        output[inside] = map_coordinates(
            distance_field,
            [rows[inside], columns[inside]],
            order=1,
            mode="constant",
            cval=5.0,
        )
        return output

    def objective(transform: np.ndarray) -> float:
        # A clipped mean is robust to boxes absent from a historical SLAM map,
        # while still requiring the transform to explain structures everywhere.
        return float(np.mean(np.minimum(distances(transform), 0.75)))

    bounds = ((-4.0, 4.0), (-4.0, 4.0), (-0.35, 0.35))
    candidates = []
    for seed in (7, 13, 29):
        result = differential_evolution(
            objective,
            bounds,
            seed=seed,
            maxiter=150,
            popsize=15,
            tol=1.0e-8,
            polish=True,
            workers=1,
        )
        candidates.append(result)
    best = min(candidates, key=lambda candidate: float(candidate.fun))
    transform = np.asarray(best.x, dtype=float)
    candidate_transforms = np.asarray([candidate.x for candidate in candidates], dtype=float)
    repeatability_spread = np.ptp(candidate_transforms, axis=0)
    fit_residuals = distances(transform)
    overall = residual_stats(fit_residuals)

    box_x = np.asarray([box.pose.x for box in boxes], dtype=float)
    one_third, two_thirds = np.quantile(box_x, [1.0 / 3.0, 2.0 / 3.0])
    region_masks = {
        "west": box_x[owners] <= one_third,
        "center": (box_x[owners] > one_third) & (box_x[owners] <= two_thirds),
        "east": box_x[owners] > two_thirds,
    }
    regions = {
        name: asdict(residual_stats(fit_residuals[mask]))
        for name, mask in region_masks.items()
    }
    worst_representative = max(region["p90_m"] for region in regions.values())
    per_box = []
    for box_index, box in enumerate(boxes):
        values = fit_residuals[owners == box_index]
        box_stats = residual_stats(values)
        per_box.append(
            {
                "name": box.name,
                "world_center_xy_m": [float(box.pose.x), float(box.pose.y)],
                "world_size_xy_m": [float(box.size[0]), float(box.size[1])],
                "median_m": box_stats.median_m,
                "p90_m": box_stats.p90_m,
                "maximum_m": box_stats.maximum_m,
            }
        )
    per_box.sort(key=lambda item: float(item["median_m"]), reverse=True)

    tx, ty, yaw = (float(value) for value in transform)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    spawn_map_x = cosine * args.spawn_x - sine * args.spawn_y + tx
    spawn_map_y = sine * args.spawn_x + cosine * args.spawn_y + ty
    spawn_map_yaw = math.atan2(
        math.sin(args.spawn_yaw + yaw), math.cos(args.spawn_yaw + yaw)
    )
    passed = (
        overall.median_m <= 0.10
        and overall.p90_m <= 0.30
        and worst_representative <= 0.30
        and bool(np.all(repeatability_spread <= np.asarray([0.002, 0.002, 0.0002])))
    )

    report = {
        "schema": "a_pipeline_map_alignment/v1",
        "status": "PASS" if passed else "FAIL",
        "tf_direction": "map_to_odom",
        "coordinate_equation": "p_map = R(yaw) * p_odom + [x, y]",
        "transform": {"x_m": tx, "y_m": ty, "yaw_rad": yaw},
        "spawn_odom_pose": {
            "x_m": args.spawn_x,
            "y_m": args.spawn_y,
            "yaw_rad": args.spawn_yaw,
        },
        "spawn_map_pose": {
            "x_m": spawn_map_x,
            "y_m": spawn_map_y,
            "yaw_rad": spawn_map_yaw,
        },
        "residuals": {
            "overall": asdict(overall),
            "regions_by_world_x_tertile": regions,
            "worst_representative_residual_m": worst_representative,
            "worst_representative_definition": "maximum P90 across west/center/east world-X box-count tertiles",
            "absolute_maximum_residual_m": overall.maximum_m,
            "largest_per_box_outliers": per_box[:10],
        },
        "acceptance": {
            "median_limit_m": 0.10,
            "p90_limit_m": 0.30,
            "each_region_p90_limit_m": 0.30,
        },
        "fit": {
            "objective": "mean(min(nearest_occupied_distance, 0.75m))",
            "bounds": [list(item) for item in bounds],
            "seeds": [7, 13, 29],
            "candidate_transforms": candidate_transforms.tolist(),
            "repeatability_peak_to_peak": repeatability_spread.tolist(),
            "region_world_x_tertile_boundaries_m": [one_third, two_thirds],
            "samples_per_edge": args.samples_per_edge,
            "sample_count": int(world_points.shape[0]),
        },
        "inputs": {
            "map_yaml": str(map_yaml),
            "map_yaml_sha256": sha256(map_yaml),
            "map_image": str(image_path),
            "map_image_sha256": sha256(image_path),
            "world": str(world),
            "world_sha256": world_hash,
            "usda": str(usda),
            "usda_sha256": sha256(usda),
            "static_box_count": len(boxes),
            "skipped_gazebo_includes": skipped_includes,
            "map_size_pixels": [width, height],
            "resolution_m_per_pixel": resolution,
            "origin": list(origin_values),
        },
        "scope_note": (
            "PASS validates a global rigid registration for the historical occupancy map. "
            "Large individual outliers remain visible in absolute_maximum_residual_m; "
            "the live global/local obstacle layers must remain enabled."
        ),
    }
    if args.output is not None:
        output = args.output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.overlay is not None:
        render_overlay(
            args.overlay.expanduser().resolve(),
            image,
            origin_values,
            resolution,
            boxes,
            transform,
        )

    print(f"MAP_ALIGNMENT={'PASS' if passed else 'FAIL'}")
    print(f"T_MAP_ODOM_X_M={tx:.9f}")
    print(f"T_MAP_ODOM_Y_M={ty:.9f}")
    print(f"T_MAP_ODOM_YAW_RAD={yaw:.9f}")
    print(f"SPAWN_MAP_X_M={spawn_map_x:.9f}")
    print(f"SPAWN_MAP_Y_M={spawn_map_y:.9f}")
    print(f"SPAWN_MAP_YAW_RAD={spawn_map_yaw:.9f}")
    print(f"MEDIAN_RESIDUAL_M={overall.median_m:.9f}")
    print(f"P90_RESIDUAL_M={overall.p90_m:.9f}")
    print(f"WORST_REPRESENTATIVE_RESIDUAL_M={worst_representative:.9f}")
    print(f"ABSOLUTE_MAX_RESIDUAL_M={overall.maximum_m:.9f}")
    return 0 if passed else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"MAP_ALIGNMENT=FAIL error={exc}", file=sys.stderr)
        raise SystemExit(2) from exc
