#!/usr/bin/env python3
"""Host CPython 3.10 adapter between JSON and HuNav ComputeAgents."""

from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from hunav_msgs.msg import Agent, AgentBehavior, Agents
from hunav_msgs.srv import ComputeAgents, ResetAgents
from std_msgs.msg import String

from hunav_protocol import COMMAND_SCHEMA, STATUS_SCHEMA, ProtocolError, validate_state


BEHAVIOR_TYPES = {
    "regular": AgentBehavior.BEH_REGULAR,
    "impassive": AgentBehavior.BEH_IMPASSIVE,
    "surprised": AgentBehavior.BEH_SURPRISED,
    "scared": AgentBehavior.BEH_SCARED,
    "curious": AgentBehavior.BEH_CURIOUS,
    "threatening": AgentBehavior.BEH_THREATENING,
}


def stamp_from_seconds(seconds: float):
    from builtin_interfaces.msg import Time
    whole = int(seconds)
    return Time(sec=whole, nanosec=int((seconds - whole) * 1_000_000_000))


def fill_agent(row: dict, agent_type: int) -> Agent:
    message = Agent()
    message.id = int(row["id"])
    message.type = agent_type
    message.name = str(row.get("name", f"agent_{message.id}"))
    message.group_id = int(row.get("group_id", -1))
    message.position.position.x = float(row["position"][0])
    message.position.position.y = float(row["position"][1])
    message.position.orientation.w = math.cos(0.5 * float(row.get("yaw", 0.0)))
    message.position.orientation.z = math.sin(0.5 * float(row.get("yaw", 0.0)))
    message.yaw = float(row.get("yaw", 0.0))
    message.velocity.linear.x = float(row["velocity"][0])
    message.velocity.linear.y = float(row["velocity"][1])
    message.linear_vel = math.hypot(message.velocity.linear.x, message.velocity.linear.y)
    message.angular_vel = float(row.get("angular_velocity", 0.0))
    message.velocity.angular.z = message.angular_vel
    message.desired_velocity = float(row.get("desired_velocity", 1.0))
    message.radius = float(row.get("radius", 0.25))
    message.goal_radius = float(row.get("goal_radius", 0.30))
    message.cyclic_goals = bool(row.get("cyclic_goals", True))
    for xy in row.get("goals", []):
        from geometry_msgs.msg import Pose
        goal = Pose()
        goal.position.x, goal.position.y = float(xy[0]), float(xy[1])
        goal.orientation.w = 1.0
        message.goals.append(goal)
    from geometry_msgs.msg import Point
    for xy in row.get("closest_obstacles", []):
        obstacle = Point()
        obstacle.x, obstacle.y = float(xy[0]), float(xy[1])
        obstacle.z = 0.0
        message.closest_obs.append(obstacle)
    behavior = row.get("behavior", {})
    if isinstance(behavior, str):
        behavior = {"type": behavior}
    behavior_name = str(behavior.get("type", "regular"))
    if behavior_name not in BEHAVIOR_TYPES:
        raise ProtocolError(f"unsupported HuNav behavior: {behavior_name}")
    message.behavior.type = BEHAVIOR_TYPES[behavior_name]
    message.behavior.configuration = AgentBehavior.BEH_CONF_CUSTOM
    message.behavior.duration = float(behavior.get("duration", 40.0))
    message.behavior.once = bool(behavior.get("once", False))
    message.behavior.vel = float(behavior.get("velocity", message.desired_velocity))
    message.behavior.dist = float(behavior.get("interaction_distance", 1.5))
    message.behavior.social_force_factor = float(behavior.get("social_force_factor", 5.0))
    message.behavior.goal_force_factor = float(behavior.get("goal_force_factor", 2.0))
    message.behavior.obstacle_force_factor = float(behavior.get("obstacle_force_factor", 10.0))
    message.behavior.other_force_factor = float(behavior.get("other_force_factor", 20.0))
    return message


def request_from_state(payload: dict) -> ComputeAgents.Request:
    state = validate_state(payload)
    request = ComputeAgents.Request()
    request.current_agents = Agents()
    request.current_agents.header.frame_id = str(state.get("frame_id", "odom"))
    request.current_agents.header.stamp = stamp_from_seconds(float(state["sim_time"]))
    request.current_agents.agents = [fill_agent(row, Agent.PERSON) for row in state["agents"]]
    robot = dict(state["robot"])
    robot.setdefault("id", 0)
    robot.setdefault("name", "isaac_mecanum")
    robot.setdefault("goals", [[robot["position"][0], robot["position"][1]]])
    request.robot = fill_agent(robot, Agent.ROBOT)
    return request


class AdapterNode:
    def __init__(self, service_name: str):
        self.node = rclpy.create_node("hunav_isaac_adapter")
        self.command_pub = self.node.create_publisher(String, "/isaac_hunav/command_json", 10)
        self.status_pub = self.node.create_publisher(String, "/isaac_hunav/status", 10)
        self.state_sub = self.node.create_subscription(String, "/isaac_hunav/state_json", self.on_state, 10)
        self.client = self.node.create_client(ComputeAgents, service_name)
        self.reset_client = self.node.create_client(
            ResetAgents, service_name.rsplit("/", 1)[0] + "/reset_agents"
        )
        self.pending = None
        self.latest_state = None
        self.command_sequence = 0
        self.last_input_sequence = -1
        self.session_id = None
        self.reset_pending = None
        self.input_count = self.output_count = self.timeout_count = self.invalid_count = 0
        self.latencies_ms: list[float] = []

    def publish(self, publisher, payload: dict) -> None:
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        publisher.publish(message)

    def on_state(self, message: String) -> None:
        try:
            state = validate_state(json.loads(message.data))
        except (ProtocolError, TypeError, ValueError):
            self.invalid_count += 1
            return
        if state["session_id"] != self.session_id:
            self.session_id = state["session_id"]
            self.last_input_sequence = -1
            self.pending = None
            if self.reset_client.service_is_ready():
                self.reset_pending = self.reset_client.call_async(ResetAgents.Request())
        elif int(state["sequence"]) <= self.last_input_sequence:
            return
        self.latest_state = state
        self.input_count += 1

    def tick(self) -> None:
        if self.reset_pending is not None:
            if not self.reset_pending.done():
                return
            self.reset_pending = None
        if self.pending is not None:
            future, state, started = self.pending
            if future.done():
                self.pending = None
                try:
                    response = future.result()
                    self.command_sequence += 1
                    commands = []
                    source_by_id = {
                        int(row["id"]): row for row in state["agents"]
                    }
                    for agent in response.updated_agents.agents:
                        source = source_by_id[int(agent.id)]
                        if not source["goals"]:
                            raise ProtocolError(
                                f"agent {int(agent.id)} has no authoritative requested goal"
                            )
                        if not agent.goals:
                            raise ProtocolError(
                                f"HuNav agent {int(agent.id)} returned no active goal"
                            )
                        requested_goal = [
                            float(source["goals"][0][0]),
                            float(source["goals"][0][1]),
                        ]
                        active_goal = [
                            float(agent.goals[0].position.x),
                            float(agent.goals[0].position.y),
                        ]
                        goal_sync_error = math.hypot(
                            active_goal[0] - requested_goal[0],
                            active_goal[1] - requested_goal[1],
                        )
                        commands.append({
                            "id": int(agent.id),
                            "desired_velocity": [float(agent.velocity.linear.x), float(agent.velocity.linear.y)],
                            "source_position": [
                                float(source["position"][0]),
                                float(source["position"][1]),
                            ],
                            "target_position": [float(agent.position.position.x), float(agent.position.position.y)],
                            "yaw": float(agent.yaw),
                            "behavior_state": int(agent.behavior.state),
                            "requested_goal": requested_goal,
                            "active_goal": active_goal,
                            "active_goal_count": len(agent.goals),
                            "goal_sync_error_m": goal_sync_error,
                        })
                    expected_ids = {int(row["id"]) for row in state["agents"]}
                    returned_ids = {int(row["id"]) for row in commands}
                    if returned_ids != expected_ids:
                        raise ProtocolError(
                            f"joint ComputeAgents response ids {sorted(returned_ids)} "
                            f"do not match request ids {sorted(expected_ids)}"
                        )
                    payload = {
                        "schema": COMMAND_SCHEMA,
                        "session_id": state["session_id"],
                        "sequence": self.command_sequence,
                        "state_sequence": int(state["sequence"]),
                        "sim_time": float(state["sim_time"]),
                        "commands": commands,
                    }
                    self.publish(self.command_pub, payload)
                    self.output_count += 1
                    self.latencies_ms.append((time.monotonic() - started) * 1000.0)
                    self.last_input_sequence = int(state["sequence"])
                except Exception:
                    self.invalid_count += 1
            elif time.monotonic() - started > 0.30:
                self.timeout_count += 1
            return
        if self.latest_state is not None and int(self.latest_state["sequence"]) > self.last_input_sequence:
            state = self.latest_state
            self.pending = (self.client.call_async(request_from_state(state)), state, time.monotonic())

    def status(self) -> None:
        latency = sum(self.latencies_ms[-100:]) / max(len(self.latencies_ms[-100:]), 1)
        self.publish(self.status_pub, {
            "schema": STATUS_SCHEMA,
            "manager_connected": self.client.service_is_ready(),
            "input_count": self.input_count,
            "output_count": self.output_count,
            "timeout_count": self.timeout_count,
            "invalid_count": self.invalid_count,
            "mean_service_latency_ms": latency,
        })


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--service", default="/isaac_hunav/compute_agents")
    args = parser.parse_args()
    rclpy.init()
    adapter = AdapterNode(args.service)
    adapter.node.create_timer(0.01, adapter.tick)
    adapter.node.create_timer(1.0, adapter.status)
    try:
        rclpy.spin(adapter.node)
    finally:
        adapter.node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
