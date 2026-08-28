#!/usr/bin/env python3
"""Convert Isaac Sim pedestrian JSON into the project's typed ROS 2 message."""

import json
import math

import rclpy
from rclpy.node import Node
from semantic_nav_gazebo.msg import PedestrianState, PedestrianStateArray
from std_msgs.msg import String


class IsaacPedestrianGroundTruthAdapter(Node):
    def __init__(self) -> None:
        super().__init__("isaac_pedestrian_ground_truth_adapter")
        self.input_topic = str(
            self.declare_parameter(
                "input_topic", "/isaac_sim/pedestrian_ground_truth_json"
            ).value
        )
        self.output_topic = str(
            self.declare_parameter("output_topic", "/pedestrian_ground_truth").value
        )
        self.publisher = self.create_publisher(
            PedestrianStateArray, self.output_topic, 10
        )
        self.subscription = self.create_subscription(
            String, self.input_topic, self.convert, 10
        )
        self.publish_count = 0
        self.get_logger().info(
            f"Isaac pedestrian adapter ready: {self.input_topic} -> {self.output_topic}"
        )

    def convert(self, source: String) -> None:
        try:
            payload = json.loads(source.data)
            sim_time = max(0.0, float(payload["sim_time"]))
            pedestrians = payload["pedestrians"]
            if not isinstance(pedestrians, list):
                raise ValueError("pedestrians must be a list")

            output = PedestrianStateArray()
            whole_seconds = int(sim_time)
            output.header.stamp.sec = whole_seconds
            output.header.stamp.nanosec = int(
                (sim_time - whole_seconds) * 1_000_000_000
            )
            output.header.frame_id = str(payload.get("frame_id", "odom"))

            for item in pedestrians:
                position = item["position"]
                velocity = item["velocity"]
                yaw = float(item.get("yaw", 0.0))
                state = PedestrianState()
                state.id = str(item["id"])
                state.pose.position.x = float(position[0])
                state.pose.position.y = float(position[1])
                state.pose.position.z = float(position[2]) if len(position) > 2 else 0.0
                state.pose.orientation.z = math.sin(0.5 * yaw)
                state.pose.orientation.w = math.cos(0.5 * yaw)
                state.velocity.linear.x = float(velocity[0])
                state.velocity.linear.y = float(velocity[1])
                state.velocity.linear.z = float(velocity[2]) if len(velocity) > 2 else 0.0
                output.pedestrians.append(state)

            self.publisher.publish(output)
            self.publish_count += 1
            if self.publish_count % 100 == 0:
                self.get_logger().info(
                    f"published {self.publish_count} pedestrian frames; "
                    f"current agents={len(output.pedestrians)}"
                )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning(f"discarding invalid pedestrian JSON: {error}")


def main() -> None:
    rclpy.init()
    node = IsaacPedestrianGroundTruthAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
