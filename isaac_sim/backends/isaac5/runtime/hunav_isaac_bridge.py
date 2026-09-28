#!/usr/bin/env python3
"""ROS String bridge used inside Isaac's CPython 3.11 process."""

from __future__ import annotations

import json
import time
import uuid

from hunav_protocol import CommandInbox, STATE_SCHEMA, STATUS_SCHEMA


class IsaacHuNavBridge:
    def __init__(self, node, string_type, command_timeout: float = 0.35):
        self.node = node
        self.string_type = string_type
        self.inbox = CommandInbox(timeout_sec=command_timeout)
        self.state_pub = node.create_publisher(string_type, "/isaac_hunav/state_json", 10)
        self.status_pub = node.create_publisher(string_type, "/isaac_hunav/status", 10)
        self.command_sub = node.create_subscription(
            string_type, "/isaac_hunav/command_json", self._on_command, 10
        )
        self.sequence = 0
        self.session_id = str(uuid.uuid4())
        self.current_sim_time = 0.0
        self.state_publish_count = 0
        self.last_state_wall = -1.0
        self.last_fallback_reason = None

    def _publish(self, publisher, payload: dict) -> None:
        message = self.string_type()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        publisher.publish(message)

    def _on_command(self, message) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            self.inbox.rejected_invalid += 1
            return
        self.inbox.accept(payload, self.current_sim_time, self.session_id)

    def publish_state(self, sim_time: float, robot: dict, agents: list[dict]) -> int:
        self.current_sim_time = float(sim_time)
        self.sequence += 1
        self._publish(self.state_pub, {
            "schema": STATE_SCHEMA,
            "session_id": self.session_id,
            "sequence": self.sequence,
            "sim_time": self.current_sim_time,
            "frame_id": "odom",
            "robot": robot,
            "agents": agents,
        })
        self.state_publish_count += 1
        self.last_state_wall = time.monotonic()
        return self.sequence

    def commands(self, sim_time: float) -> tuple[dict[int, dict], str | None]:
        commands, reason = self.inbox.current(sim_time)
        self.last_fallback_reason = reason
        return commands, reason

    def publish_status(self, sim_time: float, manager_connected: bool) -> None:
        self._publish(self.status_pub, {
            "schema": STATUS_SCHEMA,
            "sim_time": float(sim_time),
            "manager_connected": bool(manager_connected),
            "state_publish_count": self.state_publish_count,
            "command_accept_count": self.inbox.accepted,
            "rejected_duplicate": self.inbox.rejected_duplicate,
            "rejected_out_of_order": self.inbox.rejected_out_of_order,
            "rejected_session": self.inbox.rejected_session,
            "rejected_command_sequence": self.inbox.rejected_command_sequence,
            "rejected_state_sequence": self.inbox.rejected_state_sequence,
            "rejected_future": self.inbox.rejected_future,
            "rejected_invalid": self.inbox.rejected_invalid,
            "fallback_reason": self.last_fallback_reason,
        })
