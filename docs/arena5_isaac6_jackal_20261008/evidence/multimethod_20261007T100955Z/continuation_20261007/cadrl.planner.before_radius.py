"""GA3C-CADRL (mit-acl/cadrl_ros) wrapper for the arena_planners edge bridge.

Reconstructs, outside the upstream ROS node (``scripts/cadrl_node.py`` /
``agent.py``), the agent-relative input state that GA3C-CADRL's RNN policy
consumes, and replicates the deployed action selection. The committed checkpoint
``model/cadrl_ga3c.npz`` holds the variables extracted from the in-repo TF1
checkpoint ``checkpoints/network_01900000`` -- the one ``cadrl_node.py`` actually
loads (``nn.simple_load(.../network_01900000)``), not the higher-step
``network_02360000``, which is a different 15-action variant; ``net.py``
reimplements the LSTM + MLP forward pass in torch (no TensorFlow at run time).

Observation (mirrors ``agent.Agent.observe`` with ``MULTI_AGENT_ARCH == 'RNN'``):

    state(75) = [ num_other_agents,
                  host(4)  = [dist_to_goal, heading_to_goal_ego, pref_speed, radius],
                  other_i(7) x 10 ] with
    other_i = [ p_parallel, p_orthog,   # other pos in host ego frame (prll=>goal dir)
                v_parallel, v_orthog,    # other vel in host ego frame
                other_radius, combined_radius, dist_to_other_surface ]

The host ego frame has its x-axis (``ref_prll``) pointing from the robot toward
the goal and y-axis (``ref_orth``) 90deg left. Other agents are the pedestrians
plus laser clusters (upstream ``cadrl_node`` consumes ``~clusters`` from a laser
clustering node, so static obstacles reach the policy as zero-velocity agents).
Nearest ``MAX_NUM_OTHER_AGENTS=10`` within ``SENSING_HORIZON=8 m`` are kept, sorted
farthest->nearest (upstream order); absent slots stay zero and ``num_other_agents``
sets the RNN sequence length.

Action selection is the *deployed* GA3C behaviour (``cbComputeActionGA3C``):
``argmax`` over the policy head ``softmax(logits_p)``, then look up the discrete
action ``self.actions[argmax]``. NOTE: this is a POLICY-network argmax, not the
value-based one-step propagation of the classic ICRA'17 CADRL -- the GA3C variant
that ships these weights selects actions directly from the actor head, and that is
what is replicated here. The deployed checkpoint's policy head emits 11 logits, one
per entry in the upstream ``Actions()`` table; argmax indexes that table directly.

The chosen action is ``[speed_fraction, heading_change]``: forward speed
``v = pref_speed * fraction`` and a heading change relative to the robot's current
heading. Upstream ``update_action`` commands ``twist.linear.x = v`` and
``twist.angular.z = 2 * yaw_error`` where ``yaw_error = (heading_change + psi) -
psi = heading_change`` -- a proportional turn law with gain 2.0, *not* a
``heading_change / dt`` slew. We reproduce that as ``omega = 2 * heading_change``;
no extra diff-drive projection is done here, the bridge owns wheel-level control.
"""

from __future__ import annotations

import pathlib

import numpy as np
import torch
from arena_planners.sdk import load_manifest, main_loop

from net import (
    HOST_AGENT_OBSERVATION_LENGTH,
    MAX_NUM_OTHER_AGENTS_OBSERVED,
    OTHER_AGENT_OBSERVATION_LENGTH,
    CADRLNet,
)

_HERE = pathlib.Path(__file__).parent
_WEIGHTS = _HERE / "model" / "cadrl_ga3c.npz"

# Constants from upstream scripts/network.py:Config and cadrl_node.py.
_SENSING_HORIZON: float = 8.0       # Config.SENSING_HORIZON (m)
_ROBOT_RADIUS: float = 0.5          # veh_data['radius']
_PED_RADIUS: float = 0.3            # PED_RADIUS
_PREF_SPEED: float = 1.0            # default jackal_speed / Config host avg
_NUM_ACTIONS: int = 11              # logits_p width (deployed network_01900000)
_KP_YAW: float = 2.0               # upstream update_action: twist.angular.z = 2*yaw_error
_CLUSTER_GAP: float = 0.3           # consecutive returns farther apart than this start a new cluster
_CLUSTER_MIN_BEAMS: int = 2
_CLUSTER_RADIUS_MIN: float = 0.15
_CLUSTER_RADIUS_MAX: float = 1.0

# Upstream network.Actions().actions (11 discrete [speed_fraction, heading_change]).
_pi = np.pi
_ACTIONS = np.array(
    [
        [1.0, -_pi / 6],
        [1.0, -_pi / 12],
        [1.0, 0.0],
        [1.0, _pi / 12],
        [1.0, _pi / 6],
        [0.5, -_pi / 6],
        [0.5, 0.0],
        [0.5, _pi / 6],
        [0.0, -_pi / 6],
        [0.0, 0.0],
        [0.0, _pi / 6],
    ],
    dtype=np.float64,
)
assert _ACTIONS.shape[0] == _NUM_ACTIONS  # table must match the policy head width


def _wrap(angle: float) -> float:
    """Wrap to [-pi, pi) (util.wrap)."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def laser_clusters(ranges: np.ndarray, px: float, py: float, theta: float) -> list[tuple[float, float, float]]:
    """World-frame (x, y, radius) clusters of a canonical scan (beam i at bearing 2*pi*i/N), non-returns read as the max range."""
    r = np.asarray(ranges, dtype=np.float64)
    n = r.size
    if n < 2:
        return []
    bearings = theta + 2.0 * np.pi * np.arange(n) / n
    valid = np.isfinite(r) & (r > 0.0) & (r < _SENSING_HORIZON) & (r < r.max() * 0.999)
    xs = px + r * np.cos(bearings)
    ys = py + r * np.sin(bearings)
    clusters: list[list[int]] = []
    current: list[int] = []
    for i in range(n):
        if not valid[i]:
            if current:
                clusters.append(current)
                current = []
            continue
        if current and np.hypot(xs[i] - xs[current[-1]], ys[i] - ys[current[-1]]) > _CLUSTER_GAP:
            clusters.append(current)
            current = []
        current.append(i)
    if current:
        clusters.append(current)
    if len(clusters) > 1 and valid[0] and valid[n - 1] and np.hypot(xs[0] - xs[n - 1], ys[0] - ys[n - 1]) <= _CLUSTER_GAP:
        clusters[0] = clusters.pop() + clusters[0]
    out = []
    for idx in clusters:
        if len(idx) < _CLUSTER_MIN_BEAMS:
            continue
        cx, cy = float(xs[idx].mean()), float(ys[idx].mean())
        extent = float(np.hypot(xs[idx] - cx, ys[idx] - cy).max())
        out.append((cx, cy, float(np.clip(extent, _CLUSTER_RADIUS_MIN, _CLUSTER_RADIUS_MAX))))
    return out


class _Runner:
    def __init__(self) -> None:
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.net = CADRLNet(num_actions=_NUM_ACTIONS).load_npz(str(_WEIGHTS)).to(self.device)
        self.net.eval()

    def build_state(
        self,
        px: float,
        py: float,
        theta: float,
        vx: float,
        vy: float,
        gx: float,
        gy: float,
        others: list[tuple[float, float, float, float, float]],
    ) -> np.ndarray:
        """Build the 75-d GA3C-CADRL host-frame state (agent.Agent.observe) from (x, y, vx, vy, radius) others."""
        state = np.zeros(
            1 + HOST_AGENT_OBSERVATION_LENGTH + MAX_NUM_OTHER_AGENTS_OBSERVED * OTHER_AGENT_OBSERVATION_LENGTH,
            dtype=np.float32,
        )

        # host ego frame: ref_prll points robot -> goal, ref_orth is +90deg.
        goal_dir = np.array([gx - px, gy - py], dtype=np.float64)
        dist_to_goal = float(np.linalg.norm(goal_dir))
        if dist_to_goal > 1e-8:
            ref_prll = goal_dir / dist_to_goal
        else:
            ref_prll = goal_dir
        ref_orth = np.array([-ref_prll[1], ref_prll[0]])

        ref_prll_angle = np.arctan2(ref_prll[1], ref_prll[0])
        heading_ego = _wrap(theta - ref_prll_angle)

        # host block: [dist_to_goal, heading_to_goal(ego), pref_speed, radius]
        state[1 + 0] = dist_to_goal
        state[1 + 1] = heading_ego
        state[1 + 2] = _PREF_SPEED
        state[1 + 3] = _ROBOT_RADIUS

        if not others:
            state[0] = 0
            return state

        # gather (dist_2_other_surface, px, py, vx, vy, radius) for in-range others
        candidates = []
        for opx, opy, ovx, ovy, oradius in others:
            rel = np.array([opx - px, opy - py], dtype=np.float64)
            dist_centers = float(np.linalg.norm(rel))
            if dist_centers > _SENSING_HORIZON:
                continue
            dist_surface = dist_centers - _ROBOT_RADIUS - oradius
            candidates.append((dist_surface, opx, opy, ovx, ovy, oradius))

        # upstream: sort by surface distance ascending, reverse, keep last 10
        # (== keep the 10 nearest, ordered farthest -> nearest).
        candidates.sort(key=lambda c: c[0])
        candidates.reverse()
        clipped = candidates[-MAX_NUM_OTHER_AGENTS_OBSERVED:]

        i = 0
        for dist_surface, opx, opy, ovx, ovy, oradius in clipped:
            rel = np.array([opx - px, opy - py], dtype=np.float64)
            vel = np.array([ovx, ovy], dtype=np.float64)
            p_parallel = float(np.dot(rel, ref_prll))
            p_orthog = float(np.dot(rel, ref_orth))
            v_parallel = float(np.dot(vel, ref_prll))
            v_orthog = float(np.dot(vel, ref_orth))
            combined_radius = _ROBOT_RADIUS + oradius
            s = 1 + HOST_AGENT_OBSERVATION_LENGTH + OTHER_AGENT_OBSERVATION_LENGTH * i
            state[s : s + OTHER_AGENT_OBSERVATION_LENGTH] = (
                p_parallel,
                p_orthog,
                v_parallel,
                v_orthog,
                oradius,
                combined_radius,
                dist_surface,
            )
            i += 1

        state[0] = i  # RNN sequence length
        return state

    @torch.no_grad()
    def act(self, state: np.ndarray, theta: float) -> tuple[float, float]:
        """Policy-argmax action -> (forward speed v, heading change)."""
        x = torch.from_numpy(state).unsqueeze(0).to(self.device)
        policy = self.net.predict_p(x)[0].cpu().numpy()
        best = int(np.argmax(policy))
        speed_fraction, heading_change = _ACTIONS[best]
        v = float(_PREF_SPEED * speed_fraction)
        return v, float(heading_change)


_runner: _Runner | None = None


def _get_runner() -> _Runner:
    global _runner
    if _runner is None:
        _runner = _Runner()
    return _runner


def _others(peds: np.ndarray | None, ranges: np.ndarray | None, px: float, py: float, theta: float) -> list[tuple[float, float, float, float, float]]:
    """Pedestrians as moving agents plus laser clusters as static ones, clusters that coincide with a pedestrian dropped."""
    out: list[tuple[float, float, float, float, float]] = []
    if peds is not None:
        for row in peds:
            ovx = float(row[3]) if len(row) > 3 else 0.0
            ovy = float(row[4]) if len(row) > 4 else 0.0
            out.append((float(row[1]), float(row[2]), ovx, ovy, _PED_RADIUS))
    if ranges is None:
        return out
    peds_xy = [(ox, oy) for ox, oy, _vx, _vy, _r in out]
    for cx, cy, cr in laser_clusters(ranges, px, py, theta):
        if any(np.hypot(cx - ox, cy - oy) <= cr + _PED_RADIUS for ox, oy in peds_xy):
            continue
        out.append((cx, cy, 0.0, 0.0, cr))
    return out


def step(features: dict) -> list[float]:
    """Map the bridge feature dict to a differential-drive [v, omega] twist."""
    runner = _get_runner()

    robot_pose = features.get("robot_pose")
    goal_pose = features.get("goal_pose")
    if robot_pose is None or goal_pose is None:
        return [0.0, 0.0]

    px, py, theta = float(robot_pose[0]), float(robot_pose[1]), float(robot_pose[2])
    gx, gy = float(goal_pose[0]), float(goal_pose[1])

    robot_state = features.get("robot_state")
    if robot_state is not None and len(robot_state) >= 4:
        vx, vy = float(robot_state[2]), float(robot_state[3])
    else:
        vx, vy = 0.0, 0.0

    others = _others(features.get("pedestrians"), features.get("laser_scan"), px, py, theta)

    state = runner.build_state(px, py, theta, vx, vy, gx, gy, others)
    v, heading_change = runner.act(state, theta)

    # upstream update_action: omega = 2 * yaw_error, with yaw_error == heading_change.
    omega = _KP_YAW * heading_change
    return [v, omega]


def on_reset(episode_id: str, initial_state: dict | None) -> None:
    # The LSTM is unrolled fresh over the other-agent set each step (no carried
    # hidden state across control steps), so there is nothing to reset.
    _get_runner()


if __name__ == "__main__":
    manifest = load_manifest(_HERE / "planner.yaml")
    main_loop(step, manifest=manifest, on_reset=on_reset)
