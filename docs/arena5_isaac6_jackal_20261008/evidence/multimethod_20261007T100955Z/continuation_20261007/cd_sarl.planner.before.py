"""CD-SARL (curriculum + diverse-dynamics SARL) wrapper for the arena_planners bridge.

Plain SARL (Crowd-robot attention DRL) trained with curriculum learning and
diverse pedestrian dynamics in RAISE-Lab/soc-nav-training. The value-network
architecture is identical to the crowdnav planner; this adapter only differs in
the checkpoint shipped (sarl_curr_div/rl_model.pth) and its config.
"""

from __future__ import annotations

import configparser
import math
import pathlib

from arena_planners.sdk import load_manifest, main_loop
from policy import SARLPolicy
from state import FullState, JointState, ObservableState

_V_PREF: float = 1.0
_RADIUS: float = 0.3
_TIME_STEP: float = 0.25
_ABSENT_DIST: float = 15.0  # crowd_sim parks absent humans at (15, 15)

_policy: SARLPolicy | None = None


def _build_policy() -> SARLPolicy:
    config_path = pathlib.Path(__file__).parent / "configs" / "policy.config"
    config = configparser.RawConfigParser()
    config.read(str(config_path))
    p = SARLPolicy()
    p.configure(config)
    import torch

    weights = pathlib.Path(__file__).parent / "model" / "rl_model.pth"
    if not weights.exists():
        raise FileNotFoundError(f"{weights} is missing; run: arena feature planners add cd-sarl")
    p.model.load_state_dict(torch.load(str(weights), map_location="cpu"), strict=True)
    p.set_device("cpu")
    p.set_phase("test")
    p.time_step = _TIME_STEP
    p.query_env = False
    return p


def step(features: dict) -> list[float]:
    """Map features dict to CD-SARL action, return [vx, vy] (holonomic)."""
    global _policy
    if _policy is None:
        _policy = _build_policy()

    robot_pose = features.get("robot_pose")
    robot_state = features.get("robot_state")
    if robot_pose is None or robot_state is None:
        return [0.0, 0.0]
    px, py, theta = float(robot_pose[0]), float(robot_pose[1]), float(robot_pose[2])
    bvx, bvy = float(robot_state[2]), float(robot_state[3])
    c, s = math.cos(theta), math.sin(theta)
    vx, vy = c * bvx - s * bvy, s * bvx + c * bvy

    goal_pose = features.get("goal_pose")
    target: tuple[float, float] | None = None
    if goal_pose is not None:
        target = (float(goal_pose[0]), float(goal_pose[1]))
    if target is None:
        return [0.0, 0.0]
    gx, gy = target

    self_state = FullState(px, py, vx, vy, _RADIUS, gx, gy, _V_PREF, theta)

    human_states = []
    peds = features.get("pedestrians")
    if peds is not None:
        for ped in peds:
            human_states.append(
                ObservableState(
                    float(ped[1]),
                    float(ped[2]),
                    float(ped[3]),
                    float(ped[4]),
                    _RADIUS,
                )
            )

    if not human_states:
        # SARL's attention layer is undefined over an empty crowd, and upstream
        # crowd_sim never produces one. Pad with a stationary human far outside the
        # sensing horizon, matching how dsrnn fills absent humans, so the policy
        # still runs instead of being replaced by straight-line pursuit.
        human_states.append(ObservableState(px + _ABSENT_DIST, py + _ABSENT_DIST, 0.0, 0.0, _RADIUS))

    joint_state = JointState(self_state, human_states)
    action = _policy.predict(joint_state)
    return [float(action.vx), float(action.vy)]


def on_reset(episode_id: str, initial_state: dict | None) -> None:
    global _policy
    if _policy is not None:
        _policy.action_space = None


if __name__ == "__main__":
    manifest = load_manifest(pathlib.Path(__file__).parent / "planner.yaml")
    main_loop(step, manifest=manifest, on_reset=on_reset)
