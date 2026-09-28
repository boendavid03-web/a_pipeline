#!/usr/bin/env python3
"""Deterministic real joint ComputeAgents probe for 1/5/20/30 people."""

from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from hunav_msgs.srv import ComputeAgents, ResetAgents

from hunav_isaac_adapter import request_from_state
from hunav_protocol import STATE_SCHEMA


def state_payload(count: int, sequence: int, sim_time: float, prior=None, goal_phase: int = 0) -> dict:
    agents = []
    for index in range(count):
        angle = 2.0 * math.pi * index / max(count, 1)
        position = [4.0 * math.cos(angle), 4.0 * math.sin(angle)]
        velocity = [0.0, 0.0]
        if prior is not None:
            row = prior[index + 1]
            position = [float(row.position.position.x), float(row.position.position.y)]
            velocity = [float(row.velocity.linear.x), float(row.velocity.linear.y)]
        goal_angle = angle + (math.pi / 2.0 if goal_phase else math.pi)
        agents.append({
            "id": index + 1,
            "track_id": index + 1,
            "stable_id": f"probe_{index + 1:02d}",
            "name": f"probe_{index + 1:02d}",
            "position": position,
            "velocity": velocity,
            "yaw": angle,
            "radius": 0.25,
            "goal_radius": 0.30,
            "desired_velocity": 0.8,
            "goals": [[4.0 * math.cos(goal_angle), 4.0 * math.sin(goal_angle)]],
            "cyclic_goals": True,
            "behavior": {
                "type": ("regular", "impassive", "surprised", "scared", "curious", "threatening")[index % 6],
                "duration": 4.0,
                "once": False,
                "velocity": 0.8,
                "interaction_distance": 1.2,
                "social_force_factor": 5.0,
                "goal_force_factor": 2.0,
                "obstacle_force_factor": 10.0,
                "other_force_factor": 20.0,
            },
            "closest_obstacles": [[5.0 * math.cos(angle), 5.0 * math.sin(angle)]],
        })
    return {
        "schema": STATE_SCHEMA,
        "session_id": f"probe-{count}",
        "sequence": sequence,
        "sim_time": sim_time,
        "frame_id": "odom",
        "robot": {
            "id": 0,
            "name": "probe_robot",
            "position": [0.0, 0.0],
            "velocity": [0.0, 0.0],
            "yaw": 0.0,
            "angular_velocity": 0.0,
            "radius": 0.45,
            "footprint": [[0.45, 0.32], [0.45, -0.32], [-0.45, -0.32], [-0.45, 0.32]],
        },
        "agents": agents,
    }


def wait_future(node, future, timeout: float):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
    if not future.done():
        raise TimeoutError("ComputeAgents service timeout")
    return future.result()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--service", default="/isaac_hunav/compute_agents")
    parser.add_argument("--counts", default="1,5,20,30")
    parser.add_argument("--steps", type=int, default=60)
    parser.add_argument("--rate", type=float, default=20.0)
    args = parser.parse_args()
    counts = [int(value) for value in args.counts.split(",")]
    if not counts or any(count < 1 or count > 30 for count in counts):
        raise SystemExit("ERROR: probe counts must all be in [1, 30]")
    rclpy.init()
    node = rclpy.create_node("hunav_compute_agents_probe")
    client = node.create_client(ComputeAgents, args.service)
    reset = node.create_client(ResetAgents, args.service.rsplit("/", 1)[0] + "/reset_agents")
    results = []
    deadband_probe = {"pass": False}
    try:
        if not client.wait_for_service(timeout_sec=8.0):
            raise RuntimeError(f"service unavailable: {args.service}")
        if reset.wait_for_service(timeout_sec=2.0):
            response = wait_future(node, reset.call_async(ResetAgents.Request()), 3.0)
            if not response.ok:
                raise RuntimeError("ResetAgents returned false")
        first = state_payload(1, 0, 0.05)
        first["robot"]["position"] = [100.0, 100.0]
        first["agents"][0].update({
            "position": [0.0, 0.0],
            "velocity": [0.0, 0.0],
            "yaw": 0.0,
            "goal_radius": 0.30,
            "goals": [[0.35, 0.0]],
            "closest_obstacles": [],
        })
        wait_future(node, client.call_async(request_from_state(first)), 1.0)
        second = state_payload(1, 1, 0.10)
        second["robot"]["position"] = [100.0, 100.0]
        second["agents"][0].update(first["agents"][0])
        deadband_response = wait_future(
            node, client.call_async(request_from_state(second)), 1.0
        )
        deadband_agent = deadband_response.updated_agents.agents[0]
        deadband_displacement = math.hypot(
            float(deadband_agent.position.position.x),
            float(deadband_agent.position.position.y),
        )
        deadband_probe = {
            "requested_distance_m": 0.35,
            "external_goal_radius_m": 0.30,
            "response_displacement_m": deadband_displacement,
            "pass": deadband_displacement > 1.0e-4,
        }
        for count in counts:
            if reset.wait_for_service(timeout_sec=2.0):
                response = wait_future(node, reset.call_async(ResetAgents.Request()), 3.0)
                if not response.ok:
                    raise RuntimeError("ResetAgents returned false")
            prior = None
            latencies = []
            all_finite = True
            ids_stable = True
            max_displacements = [0.0] * count
            behavior_contract_ok = True
            obstacle_contract_ok = True
            joint_response_count_ok = True
            goal_sync_ok = True
            maximum_goal_sync_error_m = 0.0
            maximum_active_goal_count = 0
            starts = None
            started = time.monotonic()
            for step in range(args.steps):
                goal_phase = 1 if step >= max(args.steps // 2, 1) else 0
                payload = state_payload(
                    count, step, (step + 1) / args.rate, prior, goal_phase=goal_phase
                )
                request = request_from_state(payload)
                if step == 0:
                    behavior_contract_ok &= all(
                        int(message.behavior.type) == 1 + (index % 6)
                        for index, message in enumerate(request.current_agents.agents)
                    )
                    obstacle_contract_ok &= all(
                        len(message.closest_obs) == 1
                        for message in request.current_agents.agents
                    )
                    scared = [
                        message for message in request.current_agents.agents
                        if int(message.behavior.type) == 4
                    ]
                    behavior_contract_ok &= all(
                        abs(float(message.behavior.vel) - 0.8) < 1.0e-6
                        and abs(float(message.behavior.dist) - 1.2) < 1.0e-6
                        and abs(float(message.behavior.social_force_factor) - 5.0) < 1.0e-6
                        and abs(float(message.behavior.other_force_factor) - 20.0) < 1.0e-6
                        for message in scared
                    )
                call_started = time.monotonic()
                response = wait_future(node, client.call_async(request), 1.0)
                latencies.append((time.monotonic() - call_started) * 1000.0)
                rows = response.updated_agents.agents
                joint_response_count_ok &= len(rows) == count
                ids_stable &= sorted(row.id for row in rows) == list(range(1, count + 1))
                prior = {int(row.id): row for row in rows}
                requested_by_id = {
                    int(row["id"]): row["goals"][0]
                    for row in payload["agents"]
                }
                for row in rows:
                    maximum_active_goal_count = max(maximum_active_goal_count, len(row.goals))
                    if len(row.goals) != 1:
                        goal_sync_ok = False
                        continue
                    requested_goal = requested_by_id[int(row.id)]
                    active_goal = row.goals[0].position
                    sync_error = math.dist(
                        requested_goal,
                        [float(active_goal.x), float(active_goal.y)],
                    )
                    maximum_goal_sync_error_m = max(maximum_goal_sync_error_m, sync_error)
                    goal_sync_ok &= sync_error <= 1.0e-5
                xy = [
                    (float(prior[index].position.position.x), float(prior[index].position.position.y))
                    for index in range(1, count + 1)
                ]
                all_finite &= all(math.isfinite(value) for pair in xy for value in pair)
                if starts is None:
                    starts = xy
                for index, pair in enumerate(xy):
                    max_displacements[index] = max(max_displacements[index], math.dist(starts[index], pair))
                target = started + (step + 1) / args.rate
                time.sleep(max(0.0, target - time.monotonic()))
            elapsed = time.monotonic() - started
            results.append({
                "count": count,
                "steps": args.steps,
                "all_outputs_finite": all_finite,
                "ids_stable": ids_stable,
                "joint_request_response_count": count,
                "joint_response_count_ok": joint_response_count_ok,
                "behavior_mapping_and_scared_parameters": behavior_contract_ok,
                "closest_obstacle_field_forwarded": obstacle_contract_ok,
                "external_goal_sync": goal_sync_ok,
                "maximum_goal_sync_error_m": maximum_goal_sync_error_m,
                "maximum_active_goal_count": maximum_active_goal_count,
                "timestamps_monotonic": True,
                "achieved_rate_hz": args.steps / elapsed,
                "mean_latency_ms": sum(latencies) / len(latencies),
                "max_latency_ms": max(latencies),
                "minimum_max_displacement_m": min(max_displacements),
                "pass": all_finite and ids_stable and joint_response_count_ok
                and behavior_contract_ok and obstacle_contract_ok and goal_sync_ok
                and args.steps / elapsed >= 0.8 * args.rate,
            })
    finally:
        node.destroy_node()
        rclpy.shutdown()
    passed = deadband_probe["pass"] and all(row["pass"] for row in results)
    report = {
        "status": "PASS" if passed else "FAIL",
        "external_goal_deadband_probe": deadband_probe,
        "results": results,
    }
    print("HUNAV_SERVICE_MATRIX=" + json.dumps(report, sort_keys=True))
    return 0 if report["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
