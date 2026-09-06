"""Pure shared-social integration, Isaac adaptation, and quality metrics.

The module deliberately has no Isaac Sim or ROS dependency.  Its raw layer
loads the project Gazebo social kernel from the single source file used by the
Gazebo controller.  Isaac-only smoothing and steering limits are applied only
after that raw Gazebo velocity has been retained in the output contract.
One tracker ``update`` call represents one sampled simulation frame.
"""

from __future__ import annotations

from dataclasses import dataclass
import importlib.util
import math
from pathlib import Path
import sys
from typing import Mapping, Sequence


Pair = tuple[str, str]
Vector2 = tuple[float, float]


PROJECT_ROOT = Path(__file__).resolve().parents[2]
GAZEBO_SOCIAL_KERNEL_PATH = (
    PROJECT_ROOT
    / "workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/gazebo_social_kernel.py"
).resolve()
_KERNEL_MODULE_NAME = "a_pipeline_shared_gazebo_social_kernel"


def _load_shared_gazebo_social_kernel():
    """Load the exact project source file without relying on ambient PYTHONPATH."""

    if not GAZEBO_SOCIAL_KERNEL_PATH.is_file():
        raise ImportError(
            f"shared Gazebo social kernel is missing: {GAZEBO_SOCIAL_KERNEL_PATH}"
        )
    existing = sys.modules.get(_KERNEL_MODULE_NAME)
    if existing is not None:
        loaded_path = Path(existing.__file__).resolve()
        if loaded_path != GAZEBO_SOCIAL_KERNEL_PATH:
            raise ImportError(
                "shared Gazebo social kernel module resolved to the wrong file: "
                f"expected={GAZEBO_SOCIAL_KERNEL_PATH}, actual={loaded_path}"
            )
        return existing
    spec = importlib.util.spec_from_file_location(
        _KERNEL_MODULE_NAME, GAZEBO_SOCIAL_KERNEL_PATH
    )
    if spec is None or spec.loader is None:
        raise ImportError(
            f"could not load shared Gazebo social kernel: {GAZEBO_SOCIAL_KERNEL_PATH}"
        )
    module = importlib.util.module_from_spec(spec)
    sys.modules[_KERNEL_MODULE_NAME] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        sys.modules.pop(_KERNEL_MODULE_NAME, None)
        raise
    return module


_GAZEBO_KERNEL = _load_shared_gazebo_social_kernel()
KinematicState = _GAZEBO_KERNEL.KinematicState
compute_social_interaction_core = _GAZEBO_KERNEL.compute_social_interaction_core
velocity_step = _GAZEBO_KERNEL.velocity_step


def gazebo_social_kernel_source_path() -> Path:
    """Return the verified source-of-truth file used by the Isaac raw layer."""

    return Path(_GAZEBO_KERNEL.__file__).resolve()


@dataclass(frozen=True)
class PedestrianMotionState:
    """Planar state and current patrol intent for one pedestrian."""

    position_m: Vector2
    velocity_mps: Vector2
    desired_direction: Vector2
    preferred_speed_mps: float
    yaw_rad: float | None = None


@dataclass(frozen=True)
class RobotMotionState:
    """Robot-center social state plus separate Isaac collision geometry."""

    position_m: Vector2
    velocity_mps: Vector2
    yaw_rad: float
    half_extents_m: Vector2
    footprint_center_m: Vector2 | None = None


@dataclass(frozen=True)
class SocialForceParameters:
    """Shared-kernel parameters plus explicitly Isaac-only adapter limits."""

    neighbor_range_m: float = 10.0
    relaxation_time_sec: float = 0.5
    human_social_force_weight: float = 5.1
    robot_social_force_weight: float = 5.1
    robot_personal_space_force_weight: float = 6.0
    agent_radius_m: float = 0.35
    robot_radius_m: float = 0.47
    robot_clearance_m: float = 1.0
    robot_personal_space_sigma_m: float = 0.2
    smoothing_time_sec: float = 0.35
    max_total_social_accel_mps2: float = 4.0
    max_speed_correction_mps: float = 0.65
    max_lateral_speed_mps: float = 0.45
    max_steering_angle_rad: float = math.radians(35.0)
    minimum_command_speed_mps: float = 0.15
    maximum_dt_sec: float = 0.2

    def __post_init__(self) -> None:
        positive = (
            "neighbor_range_m",
            "relaxation_time_sec",
            "agent_radius_m",
            "robot_radius_m",
            "robot_clearance_m",
            "robot_personal_space_sigma_m",
            "smoothing_time_sec",
            "max_total_social_accel_mps2",
            "max_speed_correction_mps",
            "max_lateral_speed_mps",
            "max_steering_angle_rad",
            "maximum_dt_sec",
        )
        for name in positive:
            _finite_positive(name, getattr(self, name))
        nonnegative = (
            "human_social_force_weight",
            "robot_social_force_weight",
            "robot_personal_space_force_weight",
            "minimum_command_speed_mps",
        )
        for name in nonnegative:
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        if self.robot_clearance_m < self.robot_radius_m + self.agent_radius_m:
            raise ValueError(
                "robot_clearance_m must be at least robot_radius_m + agent_radius_m"
            )
        if self.max_steering_angle_rad >= 0.5 * math.pi:
            raise ValueError("max_steering_angle_rad must be smaller than pi/2")
        if not math.isclose(
            self.robot_social_force_weight,
            self.human_social_force_weight,
            rel_tol=0.0,
            abs_tol=0.0,
        ):
            raise ValueError(
                "shared Gazebo kernel requires robot_social_force_weight to "
                "equal human_social_force_weight"
            )


@dataclass(frozen=True)
class SocialMotionOutput:
    """Raw shared-kernel result followed by the Isaac actuator result."""

    desired_component_mps: Vector2
    gazebo_desired_acceleration_mps2: Vector2
    gazebo_human_social_raw_mps2: Vector2
    gazebo_human_social_weighted_mps2: Vector2
    gazebo_robot_social_raw_mps2: Vector2
    gazebo_robot_social_weighted_mps2: Vector2
    gazebo_robot_personal_space_raw_mps2: Vector2
    gazebo_robot_personal_space_weighted_mps2: Vector2
    gazebo_core_acceleration_mps2: Vector2
    gazebo_raw_pre_clip_velocity_mps: Vector2
    gazebo_raw_velocity_mps: Vector2
    gazebo_raw_dt_sec: float
    isaac_adapter_dt_sec: float
    isaac_adapter_input_velocity_mps: Vector2
    isaac_adapter_output_velocity_mps: Vector2
    adapter_forward_component_mps: float
    adapter_lateral_component_mps: float
    adapter_dt_limited: bool
    adapter_acceleration_limited: bool
    adapter_correction_limited: bool
    adapter_lateral_limited: bool
    adapter_angle_limited: bool
    adapter_smoothing_applied: bool
    applied_social_accel_mps2: Vector2
    speed_command_mps: float
    robot_footprint_clearance_m: float | None
    robot_personal_space_violation: bool

    # Compatibility aliases for existing evaluators.  Their meanings are now
    # unambiguous: weighted kernel terms and the post-kernel Isaac command.
    @property
    def human_social_component_mps2(self) -> Vector2:
        return self.gazebo_human_social_weighted_mps2

    @property
    def robot_social_component_mps2(self) -> Vector2:
        return self.gazebo_robot_social_weighted_mps2

    @property
    def robot_personal_space_component_mps2(self) -> Vector2:
        return self.gazebo_robot_personal_space_weighted_mps2

    @property
    def final_desired_velocity_mps(self) -> Vector2:
        return self.isaac_adapter_output_velocity_mps


def resolve_social_mode(value: str) -> str:
    """Validate the opt-in controller mode without depending on Isaac."""

    normalized = str(value).strip().lower()
    if normalized not in {"legacy", "gazebo_social"}:
        raise ValueError("social mode must be legacy or gazebo_social")
    return normalized


def oriented_box_clearance(
    point_m: Sequence[float],
    center_m: Sequence[float],
    yaw_rad: float,
    half_extents_m: Sequence[float],
) -> tuple[float, Vector2, Vector2]:
    """Return signed OBB clearance, outward normal, and nearest footprint point.

    Positive clearance is outside the footprint, zero is on its boundary, and
    negative clearance is inside.  The returned normal always points from the
    robot footprint toward the pedestrian, including for interior points.
    """

    px, py = _finite_vector2("point_m", point_m)
    cx, cy = _finite_vector2("center_m", center_m)
    half_x, half_y = _finite_vector2("half_extents_m", half_extents_m)
    if half_x <= 0.0 or half_y <= 0.0:
        raise ValueError("half_extents_m must be positive")
    yaw = float(yaw_rad)
    if not math.isfinite(yaw):
        raise ValueError("yaw_rad must be finite")
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rel_x, rel_y = px - cx, py - cy
    local_x = cosine * rel_x + sine * rel_y
    local_y = -sine * rel_x + cosine * rel_y
    clamped_x = max(-half_x, min(half_x, local_x))
    clamped_y = max(-half_y, min(half_y, local_y))
    delta_x, delta_y = local_x - clamped_x, local_y - clamped_y
    outside = math.hypot(delta_x, delta_y)
    if outside > 1.0e-12:
        normal_local = (delta_x / outside, delta_y / outside)
        signed_clearance = outside
    else:
        distance_x = half_x - abs(local_x)
        distance_y = half_y - abs(local_y)
        if distance_x <= distance_y:
            sign_x = 1.0 if local_x >= 0.0 else -1.0
            clamped_x = sign_x * half_x
            normal_local = (sign_x, 0.0)
            signed_clearance = -distance_x
        else:
            sign_y = 1.0 if local_y >= 0.0 else -1.0
            clamped_y = sign_y * half_y
            normal_local = (0.0, sign_y)
            signed_clearance = -distance_y
    nearest_world = (
        cx + cosine * clamped_x - sine * clamped_y,
        cy + sine * clamped_x + cosine * clamped_y,
    )
    normal_world = (
        cosine * normal_local[0] - sine * normal_local[1],
        sine * normal_local[0] + cosine * normal_local[1],
    )
    return signed_clearance, normal_world, nearest_world


@dataclass(frozen=True)
class IsaacAdapterResult:
    input_velocity_mps: Vector2
    output_velocity_mps: Vector2
    dt_sec: float
    forward_component_mps: float
    lateral_component_mps: float
    applied_social_accel_mps2: Vector2
    dt_limited: bool
    acceleration_limited: bool
    correction_limited: bool
    lateral_limited: bool
    angle_limited: bool
    smoothing_applied: bool


class IsaacActuatorAdapter:
    """Apply only Isaac locomotion stabilizations after the Gazebo raw step."""

    def __init__(self, parameters: SocialForceParameters) -> None:
        self.parameters = parameters
        self._smoothed_velocity: dict[str, Vector2] = {}

    def reset(self) -> None:
        self._smoothed_velocity.clear()

    def retain(self, names: set[str]) -> None:
        self._smoothed_velocity = {
            name: value
            for name, value in self._smoothed_velocity.items()
            if name in names
        }

    def update(
        self,
        name: str,
        state: PedestrianMotionState,
        core,
        gazebo_raw_velocity: Vector2,
        solver_dt_sec: float,
    ) -> IsaacAdapterResult:
        parameters = self.parameters
        desired_direction = state.desired_direction
        desired_velocity = _scale(desired_direction, state.preferred_speed_mps)
        adapter_dt = min(solver_dt_sec, parameters.maximum_dt_sec)
        dt_limited = adapter_dt != solver_dt_sec

        social_accel = _add(
            core.social_weighted,
            core.robot_personal_space_weighted,
        )
        applied_social_accel = _limit_norm(
            social_accel, parameters.max_total_social_accel_mps2
        )
        acceleration_limited = not _vectors_close(
            social_accel, applied_social_accel
        )

        # The exact kernel output is always the adapter input.  Reconstruct a
        # bounded first candidate only when an Isaac acceleration or dt limit
        # is active; otherwise preserve the shared post-vmax vector verbatim.
        candidate = gazebo_raw_velocity
        if dt_limited or acceleration_limited:
            candidate = _add(
                state.velocity_mps,
                _scale(
                    _add(core.desired_weighted, applied_social_accel),
                    adapter_dt,
                ),
            )
            candidate = _limit_norm(candidate, state.preferred_speed_mps)

        raw_correction = _sub(candidate, desired_velocity)
        correction = _limit_norm(
            raw_correction, parameters.max_speed_correction_mps
        )
        correction_limited = not _vectors_close(raw_correction, correction)
        bounded_velocity = _add(desired_velocity, correction)
        lateral_direction = (-desired_direction[1], desired_direction[0])
        forward = max(0.0, _dot(bounded_velocity, desired_direction))
        forward = min(state.preferred_speed_mps, forward)

        requested_lateral = _dot(bounded_velocity, lateral_direction)
        lateral = max(
            -parameters.max_lateral_speed_mps,
            min(parameters.max_lateral_speed_mps, requested_lateral),
        )
        lateral_limited = not math.isclose(
            lateral, requested_lateral, rel_tol=0.0, abs_tol=1.0e-15
        )
        angle_lateral_limit = math.tan(
            parameters.max_steering_angle_rad
        ) * max(forward, parameters.minimum_command_speed_mps)
        before_angle_limit = lateral
        lateral = max(-angle_lateral_limit, min(angle_lateral_limit, lateral))
        angle_limited = not math.isclose(
            lateral, before_angle_limit, rel_tol=0.0, abs_tol=1.0e-15
        )

        bounded_velocity = _add(
            _scale(desired_direction, forward),
            _scale(lateral_direction, lateral),
        )
        bounded_velocity = _limit_norm(
            bounded_velocity, state.preferred_speed_mps
        )
        previous = self._smoothed_velocity.get(name, desired_velocity)
        alpha = 1.0 - math.exp(-adapter_dt / parameters.smoothing_time_sec)
        smoothed = _add(
            previous, _scale(_sub(bounded_velocity, previous), alpha)
        )
        smoothed = _limit_norm(smoothed, state.preferred_speed_mps)
        self._smoothed_velocity[name] = smoothed
        smoothing_applied = not _vectors_close(smoothed, bounded_velocity)
        return IsaacAdapterResult(
            input_velocity_mps=gazebo_raw_velocity,
            output_velocity_mps=smoothed,
            dt_sec=adapter_dt,
            forward_component_mps=_dot(smoothed, desired_direction),
            lateral_component_mps=_dot(smoothed, lateral_direction),
            applied_social_accel_mps2=applied_social_accel,
            dt_limited=dt_limited,
            acceleration_limited=acceleration_limited,
            correction_limited=correction_limited,
            lateral_limited=lateral_limited,
            angle_limited=angle_limited,
            smoothing_applied=smoothing_applied,
        )


class PedestrianSocialForceController:
    """Run the shared Gazebo kernel, then the isolated Isaac adapter."""

    def __init__(self, parameters: SocialForceParameters | None = None) -> None:
        self.parameters = parameters or SocialForceParameters()
        self.adapter = IsaacActuatorAdapter(self.parameters)
        self.update_count = 0
        self.personal_space_violation_samples = 0
        self.minimum_robot_footprint_clearance_m: float | None = None

    def reset(self) -> None:
        self.adapter.reset()
        self.update_count = 0
        self.personal_space_violation_samples = 0
        self.minimum_robot_footprint_clearance_m = None

    def update(
        self,
        pedestrians: Mapping[str, PedestrianMotionState],
        robot: RobotMotionState | None,
        dt_sec: float,
    ) -> dict[str, SocialMotionOutput]:
        states = _validated_motion_states(pedestrians)
        robot_state = _validated_robot_state(robot)
        solver_dt = float(dt_sec)
        if not math.isfinite(solver_dt) or solver_dt <= 0.0:
            raise ValueError("dt_sec must be a finite positive number")
        self.adapter.retain(set(states))
        shared_robot_state = (
            KinematicState(robot_state.position_m, robot_state.velocity_mps)
            if robot_state is not None
            else None
        )
        outputs: dict[str, SocialMotionOutput] = {}
        for name in sorted(states):
            state = states[name]
            desired_direction = state.desired_direction
            desired_velocity = _scale(
                desired_direction, state.preferred_speed_mps
            )
            pedestrian_yaw = (
                state.yaw_rad
                if state.yaw_rad is not None
                else math.atan2(desired_direction[1], desired_direction[0])
            )
            core = compute_social_interaction_core(
                pedestrian_position=state.position_m,
                pedestrian_yaw=pedestrian_yaw,
                pedestrian_velocity=state.velocity_mps,
                desired_direction=desired_direction,
                vmax=state.preferred_speed_mps,
                relaxation_time=self.parameters.relaxation_time_sec,
                desired_force_factor=1.0,
                human_neighbors=(
                    KinematicState(other.position_m, other.velocity_mps)
                    for other_name, other in sorted(states.items())
                    if other_name != name
                ),
                robot_state=shared_robot_state,
                neighbor_range=self.parameters.neighbor_range_m,
                force_social=self.parameters.human_social_force_weight,
                robot_clearance=self.parameters.robot_clearance_m,
                sigma_robot_personal_space=(
                    self.parameters.robot_personal_space_sigma_m
                ),
                force_robot_personal_space=(
                    self.parameters.robot_personal_space_force_weight
                ),
            )
            step = velocity_step(
                state.velocity_mps,
                core.core_acceleration,
                solver_dt,
                state.preferred_speed_mps,
            )
            adapter = self.adapter.update(
                name,
                state,
                core,
                step.post_clip_velocity,
                solver_dt,
            )

            # OBB geometry is retained strictly as Isaac safety/quality
            # evidence.  It cannot influence any shared force or raw velocity.
            robot_clearance = None
            robot_violation = False
            if robot_state is not None:
                robot_clearance, _outward, _nearest = oriented_box_clearance(
                    state.position_m,
                    (
                        robot_state.footprint_center_m
                        if robot_state.footprint_center_m is not None
                        else robot_state.position_m
                    ),
                    robot_state.yaw_rad,
                    robot_state.half_extents_m,
                )
                personal_clearance = max(
                    self.parameters.agent_radius_m,
                    self.parameters.robot_clearance_m
                    - self.parameters.robot_radius_m,
                )
                robot_violation = robot_clearance < personal_clearance
                if robot_violation:
                    self.personal_space_violation_samples += 1
                if (
                    self.minimum_robot_footprint_clearance_m is None
                    or robot_clearance < self.minimum_robot_footprint_clearance_m
                ):
                    self.minimum_robot_footprint_clearance_m = robot_clearance

            outputs[name] = SocialMotionOutput(
                desired_component_mps=desired_velocity,
                gazebo_desired_acceleration_mps2=core.desired_acceleration,
                gazebo_human_social_raw_mps2=core.human_social_raw,
                gazebo_human_social_weighted_mps2=core.human_social_weighted,
                gazebo_robot_social_raw_mps2=core.robot_social_raw,
                gazebo_robot_social_weighted_mps2=core.robot_social_weighted,
                gazebo_robot_personal_space_raw_mps2=(
                    core.robot_personal_space_raw
                ),
                gazebo_robot_personal_space_weighted_mps2=(
                    core.robot_personal_space_weighted
                ),
                gazebo_core_acceleration_mps2=core.core_acceleration,
                gazebo_raw_pre_clip_velocity_mps=step.pre_clip_velocity,
                gazebo_raw_velocity_mps=step.post_clip_velocity,
                gazebo_raw_dt_sec=solver_dt,
                isaac_adapter_dt_sec=adapter.dt_sec,
                isaac_adapter_input_velocity_mps=adapter.input_velocity_mps,
                isaac_adapter_output_velocity_mps=adapter.output_velocity_mps,
                adapter_forward_component_mps=adapter.forward_component_mps,
                adapter_lateral_component_mps=adapter.lateral_component_mps,
                adapter_dt_limited=adapter.dt_limited,
                adapter_acceleration_limited=adapter.acceleration_limited,
                adapter_correction_limited=adapter.correction_limited,
                adapter_lateral_limited=adapter.lateral_limited,
                adapter_angle_limited=adapter.angle_limited,
                adapter_smoothing_applied=adapter.smoothing_applied,
                applied_social_accel_mps2=adapter.applied_social_accel_mps2,
                speed_command_mps=_norm(adapter.output_velocity_mps),
                robot_footprint_clearance_m=robot_clearance,
                robot_personal_space_violation=robot_violation,
            )
        self.update_count += 1
        return outputs

    def summary(self) -> dict[str, object]:
        return {
            "update_count": self.update_count,
            "shared_gazebo_kernel_path": str(gazebo_social_kernel_source_path()),
            "raw_dt_semantics": "unclamped_social_tick_dt",
            "adapter_maximum_dt_sec": self.parameters.maximum_dt_sec,
            "personal_space_violation_samples": (
                self.personal_space_violation_samples
            ),
            "minimum_robot_footprint_clearance_m": (
                self.minimum_robot_footprint_clearance_m
            ),
        }


@dataclass(frozen=True)
class PairwiseSnapshot:
    """Pairwise measurements for one sampled frame."""

    sample_index: int
    pedestrian_count: int
    pair_count: int
    min_center_distance_m: float | None
    closest_pair: Pair | None
    visual_overlap_pairs: int
    personal_space_violation_pairs: int

    @property
    def has_visual_overlap(self) -> bool:
        return self.visual_overlap_pairs > 0

    @property
    def has_personal_space_violation(self) -> bool:
        return self.personal_space_violation_pairs > 0


@dataclass(frozen=True)
class YieldDecision:
    """Pedestrians that should begin or end a social yielding pause."""

    begin_yielding: tuple[str, ...]
    end_yielding: tuple[str, ...]
    active_yielders: tuple[str, ...]


class SocialYieldPlanner:
    """Choose one deterministic yielder before two people become too close.

    This class only makes decisions; the Isaac adapter owns animation tasks.
    A yielder remains paused until it is beyond ``resume_distance_m`` from
    every other pedestrian, preventing rapid stop/start oscillation.
    """

    def __init__(
        self,
        *,
        trigger_distance_m: float = 0.90,
        resume_distance_m: float = 1.10,
    ) -> None:
        self.trigger_distance_m = _finite_positive(
            "trigger_distance_m", trigger_distance_m
        )
        self.resume_distance_m = _finite_positive(
            "resume_distance_m", resume_distance_m
        )
        if self.resume_distance_m <= self.trigger_distance_m:
            raise ValueError(
                "resume_distance_m must be greater than trigger_distance_m"
            )
        self._active: set[str] = set()

    def update(self, positions: Mapping[str, Sequence[float]]) -> YieldDecision:
        points = _validated_planar_positions(positions)
        names = sorted(points)
        present = set(names)
        self._active.intersection_update(present)

        end: list[str] = []
        for name in sorted(self._active):
            x, y = points[name]
            if all(
                other == name
                or math.hypot(points[other][0] - x, points[other][1] - y)
                >= self.resume_distance_m
                for other in names
            ):
                self._active.remove(name)
                end.append(name)

        begin: list[str] = []
        for left_index, left_name in enumerate(names):
            left_x, left_y = points[left_name]
            for right_name in names[left_index + 1 :]:
                distance = math.hypot(
                    points[right_name][0] - left_x,
                    points[right_name][1] - left_y,
                )
                if distance >= self.trigger_distance_m:
                    continue
                if left_name in self._active or right_name in self._active:
                    continue
                # Stable ordering makes seeded runs reproducible.  The other
                # person keeps moving and native avoidance routes around the
                # now-stationary, still-visible BehaviorAgent obstacle.
                yielder = right_name
                self._active.add(yielder)
                begin.append(yielder)

        return YieldDecision(
            begin_yielding=tuple(begin),
            end_yielding=tuple(end),
            active_yielders=tuple(sorted(self._active)),
        )


class SocialQualityTracker:
    """Accumulate pairwise clearance metrics from planar pedestrian positions.

    ``positions`` passed to :meth:`update` maps stable pedestrian names to an
    ``(x, y, ...)`` sequence in metres.  A violation uses a strict ``<``
    comparison, so a pair exactly on a configured boundary is not counted.
    Pair ratios use all unique pair observations across all sampled frames as
    their denominator.  They are ``0.0`` when no pair has been observed.
    """

    def __init__(
        self,
        *,
        personal_space_m: float = 1.0,
        visual_overlap_m: float = 0.45,
    ) -> None:
        personal_space_m = _finite_positive(
            "personal_space_m", personal_space_m
        )
        visual_overlap_m = _finite_positive(
            "visual_overlap_m", visual_overlap_m
        )
        if visual_overlap_m > personal_space_m:
            raise ValueError(
                "visual_overlap_m must be less than or equal to personal_space_m"
            )

        self.personal_space_m = personal_space_m
        self.visual_overlap_m = visual_overlap_m
        self.reset()

    def reset(self) -> None:
        """Clear all accumulated samples while preserving thresholds."""

        self._sample_frames = 0
        self._pair_samples = 0
        self._min_center_distance_m: float | None = None
        self._closest_pair: Pair | None = None
        self._visual_overlap_frames = 0
        self._visual_overlap_pair_samples = 0
        self._personal_space_violation_frames = 0
        self._personal_space_violation_pair_samples = 0

    def update(
        self, positions: Mapping[str, Sequence[float]]
    ) -> PairwiseSnapshot:
        """Validate and record one frame, returning that frame's measurements.

        Validation happens before counters are changed, so a rejected frame
        never leaves the tracker partially updated.
        """

        points = _validated_planar_positions(positions)
        names = sorted(points)
        pair_count = len(names) * (len(names) - 1) // 2
        min_distance: float | None = None
        closest_pair: Pair | None = None
        visual_pairs = 0
        personal_pairs = 0

        for left_index, left_name in enumerate(names):
            left_x, left_y = points[left_name]
            for right_name in names[left_index + 1 :]:
                right_x, right_y = points[right_name]
                distance = math.hypot(right_x - left_x, right_y - left_y)
                pair = (left_name, right_name)
                if min_distance is None or distance < min_distance:
                    min_distance = distance
                    closest_pair = pair
                if distance < self.visual_overlap_m:
                    visual_pairs += 1
                if distance < self.personal_space_m:
                    personal_pairs += 1

        self._sample_frames += 1
        self._pair_samples += pair_count
        if min_distance is not None and (
            self._min_center_distance_m is None
            or min_distance < self._min_center_distance_m
        ):
            self._min_center_distance_m = min_distance
            self._closest_pair = closest_pair
        if visual_pairs:
            self._visual_overlap_frames += 1
        self._visual_overlap_pair_samples += visual_pairs
        if personal_pairs:
            self._personal_space_violation_frames += 1
        self._personal_space_violation_pair_samples += personal_pairs

        return PairwiseSnapshot(
            sample_index=self._sample_frames,
            pedestrian_count=len(names),
            pair_count=pair_count,
            min_center_distance_m=min_distance,
            closest_pair=closest_pair,
            visual_overlap_pairs=visual_pairs,
            personal_space_violation_pairs=personal_pairs,
        )

    def summary(self) -> dict[str, object]:
        """Return all accumulated metrics as a JSON-safe dictionary."""

        denominator = self._pair_samples
        return {
            "personal_space_m": self.personal_space_m,
            "visual_overlap_m": self.visual_overlap_m,
            "sample_frames": self._sample_frames,
            "pair_samples": denominator,
            "min_center_distance_m": self._min_center_distance_m,
            "closest_pair": (
                list(self._closest_pair) if self._closest_pair is not None else None
            ),
            "visual_overlap_frames": self._visual_overlap_frames,
            "visual_overlap_pair_samples": self._visual_overlap_pair_samples,
            "visual_overlap_pair_ratio": (
                self._visual_overlap_pair_samples / denominator
                if denominator
                else 0.0
            ),
            "personal_space_violation_frames": (
                self._personal_space_violation_frames
            ),
            "personal_space_violation_pair_samples": (
                self._personal_space_violation_pair_samples
            ),
            "personal_space_violation_pair_ratio": (
                self._personal_space_violation_pair_samples / denominator
                if denominator
                else 0.0
            ),
        }


def _finite_positive(name: str, value: float) -> float:
    try:
        numeric = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite positive number") from exc
    if not math.isfinite(numeric) or numeric <= 0.0:
        raise ValueError(f"{name} must be a finite positive number")
    return numeric


def _finite_vector2(name: str, value: Sequence[float]) -> Vector2:
    if isinstance(value, (str, bytes)):
        raise ValueError(f"{name} must contain finite x and y values")
    try:
        x, y = float(value[0]), float(value[1])
    except (IndexError, KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain finite x and y values") from exc
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError(f"{name} must contain finite x and y values")
    return x, y


def _validated_motion_states(
    pedestrians: Mapping[str, PedestrianMotionState],
) -> dict[str, PedestrianMotionState]:
    if not isinstance(pedestrians, Mapping):
        raise TypeError("pedestrians must be a mapping")
    validated: dict[str, PedestrianMotionState] = {}
    for name, state in pedestrians.items():
        if not isinstance(name, str) or not name:
            raise ValueError("pedestrian names must be non-empty strings")
        if not isinstance(state, PedestrianMotionState):
            raise TypeError(f"motion state for {name!r} is invalid")
        position = _finite_vector2(f"position for {name!r}", state.position_m)
        velocity = _finite_vector2(f"velocity for {name!r}", state.velocity_mps)
        direction = _finite_vector2(
            f"desired direction for {name!r}", state.desired_direction
        )
        if _norm(direction) < 1.0e-9:
            raise ValueError(f"desired direction for {name!r} must be non-zero")
        speed = float(state.preferred_speed_mps)
        if not math.isfinite(speed) or speed <= 0.0:
            raise ValueError(f"preferred speed for {name!r} must be positive")
        yaw = None if state.yaw_rad is None else float(state.yaw_rad)
        if yaw is not None and not math.isfinite(yaw):
            raise ValueError(f"yaw for {name!r} must be finite or None")
        validated[name] = PedestrianMotionState(
            position_m=position,
            velocity_mps=velocity,
            desired_direction=_unit(direction),
            preferred_speed_mps=speed,
            yaw_rad=yaw,
        )
    return validated


def _validated_robot_state(
    robot: RobotMotionState | None,
) -> RobotMotionState | None:
    if robot is None:
        return None
    if not isinstance(robot, RobotMotionState):
        raise TypeError("robot must be a RobotMotionState or None")
    position = _finite_vector2("robot position", robot.position_m)
    velocity = _finite_vector2("robot velocity", robot.velocity_mps)
    half_extents = _finite_vector2("robot half extents", robot.half_extents_m)
    if half_extents[0] <= 0.0 or half_extents[1] <= 0.0:
        raise ValueError("robot half extents must be positive")
    yaw = float(robot.yaw_rad)
    if not math.isfinite(yaw):
        raise ValueError("robot yaw must be finite")
    footprint_center = (
        None
        if robot.footprint_center_m is None
        else _finite_vector2("robot footprint center", robot.footprint_center_m)
    )
    return RobotMotionState(
        position,
        velocity,
        yaw,
        half_extents,
        footprint_center,
    )


def _add(*vectors: Vector2) -> Vector2:
    return sum(vector[0] for vector in vectors), sum(vector[1] for vector in vectors)


def _sub(left: Vector2, right: Vector2) -> Vector2:
    return left[0] - right[0], left[1] - right[1]


def _scale(vector: Vector2, factor: float) -> Vector2:
    return vector[0] * factor, vector[1] * factor


def _dot(left: Vector2, right: Vector2) -> float:
    return left[0] * right[0] + left[1] * right[1]


def _norm(vector: Vector2) -> float:
    return math.hypot(vector[0], vector[1])


def _unit(vector: Vector2) -> Vector2:
    length = _norm(vector)
    if length < 1.0e-12:
        raise ValueError("cannot normalize a zero-length vector")
    return vector[0] / length, vector[1] / length


def _limit_norm(vector: Vector2, maximum: float) -> Vector2:
    length = _norm(vector)
    if length <= maximum or length < 1.0e-12:
        return vector
    return _scale(vector, maximum / length)


def _vectors_close(left: Vector2, right: Vector2) -> bool:
    return all(
        math.isclose(left[index], right[index], rel_tol=0.0, abs_tol=1.0e-15)
        for index in range(2)
    )


def _normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _sign(value: float) -> float:
    if value > 0.0:
        return 1.0
    if value < 0.0:
        return -1.0
    return 0.0


def _validated_planar_positions(
    positions: Mapping[str, Sequence[float]],
) -> dict[str, tuple[float, float]]:
    if not isinstance(positions, Mapping):
        raise TypeError("positions must be a mapping of pedestrian names to positions")

    validated: dict[str, tuple[float, float]] = {}
    for name, position in positions.items():
        if not isinstance(name, str) or not name:
            raise ValueError("pedestrian names must be non-empty strings")
        if isinstance(position, (str, bytes)):
            raise ValueError(f"position for {name!r} must contain finite x and y values")
        try:
            x = float(position[0])
            y = float(position[1])
        except (IndexError, KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f"position for {name!r} must contain finite x and y values"
            ) from exc
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError(f"position for {name!r} must contain finite x and y values")
        validated[name] = (x, y)
    return validated
