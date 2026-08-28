#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/data_collection/episode_event, /goal_pose, /odom
# 检测到的消息类型：Odometry; PoseStamped; String
# 检测到的文件格式：JSON, WORLD
# 可能使用的关键环境变量：NEAREST, WM_DELETE_WINDOW
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 功能节点
# 推荐运行方式：ros2 run semantic_nav_gazebo episode_goal_picker.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-27 04:08:38.967814841 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:13.643741916 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/teleop_fixed_map_capture.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_goal_arrival_node.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/teleop_fixed_map_capture.launch.py（通过 ros2 launch 启动该 ROS 2 场景）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_goal_arrival_node.py（source 载入公共环境/函数/变量）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/episode_goal_picker.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/teleop_fixed_map_capture.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_goal_arrival_node.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜episode_goal_picker.py】
# 用途：ROS 2 运行节点，负责导航、感知、遥操作、数据采集或仿真辅助功能。
# 输入输出：输入为 ROS 2 参数和订阅话题；输出为发布话题、日志或实验文件。
# 关系：通常由 launch 或 pipeline 启动，依赖 ROS 2 消息及其他节点；install 下同名文件是本源文件的安装入口。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Full-map GUI for selecting the next teleop episode goal."""

from __future__ import annotations

import json
import math
from pathlib import Path
import tkinter as tk
from tkinter import messagebox

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from PIL import Image, ImageTk
from rclpy.node import Node
from std_msgs.msg import String


class EpisodeGoalPicker(Node):
    def __init__(self, root):
        super().__init__("episode_goal_picker")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter(
            "episode_event_topic", "/data_collection/episode_event"
        )
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("show_on_start", True)
        self.declare_parameter("max_display_width", 1000)
        self.declare_parameter("max_display_height", 760)
        self.declare_parameter(
            "window_title",
            "选择下一个导航目标 / Select next navigation goal",
        )
        self.declare_parameter(
            "instructions_text",
            (
                "在完整地图上单击目标，或直接输入 map 坐标。"
                "确认后窗口会隐藏，到达并保存当前 episode 后自动再次出现。"
            ),
        )
        self.declare_parameter(
            "ready_status_text",
            "rosbag 已暂停，请选择下一个目标",
        )

        self.root = root
        self.root.title(str(self.get_parameter("window_title").value))
        self.root.protocol("WM_DELETE_WINDOW", self.root.withdraw)
        self.selected_world = None
        self.robot_world = None
        self.robot_marker = None
        self.goal_marker = None

        map_yaml = Path(str(self.get_parameter("map_yaml").value))
        if not map_yaml.is_file():
            raise FileNotFoundError(f"map_yaml does not exist: {map_yaml}")
        metadata = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
        image_path = Path(str(metadata["image"]))
        if not image_path.is_absolute():
            image_path = map_yaml.parent / image_path
        occupancy_image = Image.open(image_path).convert("L")
        self.occupancy_image = occupancy_image
        image = occupancy_image.convert("RGB")
        self.image_width, self.image_height = occupancy_image.size
        self.resolution = float(metadata["resolution"])
        self.origin_x = float(metadata["origin"][0])
        self.origin_y = float(metadata["origin"][1])
        self.negate = bool(int(metadata.get("negate", 0)))
        self.occupied_thresh = float(metadata.get("occupied_thresh", 0.65))
        self.free_thresh = float(metadata.get("free_thresh", 0.196))

        max_width = int(self.get_parameter("max_display_width").value)
        max_height = int(self.get_parameter("max_display_height").value)
        scale = min(
            1.0,
            max_width / self.image_width,
            max_height / self.image_height,
        )
        self.display_width = max(1, int(round(self.image_width * scale)))
        self.display_height = max(1, int(round(self.image_height * scale)))
        resampling = getattr(Image, "Resampling", Image)
        display_image = image.resize(
            (self.display_width, self.display_height),
            resampling.NEAREST,
        )
        self.map_photo = ImageTk.PhotoImage(display_image)

        instructions = tk.Label(
            root,
            text=str(self.get_parameter("instructions_text").value),
            anchor="w",
            justify="left",
        )
        instructions.pack(fill="x", padx=8, pady=(8, 4))
        self.canvas = tk.Canvas(
            root,
            width=self.display_width,
            height=self.display_height,
            highlightthickness=1,
            highlightbackground="#606060",
        )
        self.canvas.pack(padx=8, pady=4)
        self.canvas.create_image(0, 0, image=self.map_photo, anchor="nw")
        self.canvas.bind("<Button-1>", self.canvas_click)

        controls = tk.Frame(root)
        controls.pack(fill="x", padx=8, pady=(4, 8))
        tk.Label(controls, text="goal_x:").pack(side="left")
        self.goal_x_var = tk.StringVar()
        tk.Entry(controls, textvariable=self.goal_x_var, width=12).pack(
            side="left", padx=(3, 10)
        )
        tk.Label(controls, text="goal_y:").pack(side="left")
        self.goal_y_var = tk.StringVar()
        tk.Entry(controls, textvariable=self.goal_y_var, width=12).pack(
            side="left", padx=(3, 10)
        )
        tk.Button(
            controls,
            text="发布目标并隐藏窗口",
            command=self.publish_selected_goal,
        ).pack(side="left", padx=4)
        tk.Button(controls, text="稍后选择", command=root.withdraw).pack(
            side="left", padx=4
        )
        self.status_var = tk.StringVar(value="等待选择目标")
        tk.Label(root, textvariable=self.status_var, anchor="w").pack(
            fill="x", padx=8, pady=(0, 8)
        )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("goal_topic").value),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("episode_event_topic").value),
            self.event_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_callback,
            10,
        )
        if bool(self.get_parameter("show_on_start").value):
            self.root.after(500, self.show_picker)
        else:
            self.root.withdraw()

    def canvas_to_world(self, canvas_x, canvas_y):
        image_x = float(canvas_x) * self.image_width / self.display_width
        image_y = float(canvas_y) * self.image_height / self.display_height
        world_x = self.origin_x + (image_x + 0.5) * self.resolution
        world_y = (
            self.origin_y
            + (self.image_height - 1.0 - image_y + 0.5) * self.resolution
        )
        return world_x, world_y

    def world_to_canvas(self, world_x, world_y):
        image_x = (float(world_x) - self.origin_x) / self.resolution - 0.5
        image_y = (
            self.image_height
            - 1.0
            - ((float(world_y) - self.origin_y) / self.resolution - 0.5)
        )
        return (
            image_x * self.display_width / self.image_width,
            image_y * self.display_height / self.image_height,
        )

    def world_to_map_pixel(self, world_x, world_y):
        grid_x = math.floor((float(world_x) - self.origin_x) / self.resolution)
        grid_y = math.floor((float(world_y) - self.origin_y) / self.resolution)
        if (
            grid_x < 0
            or grid_x >= self.image_width
            or grid_y < 0
            or grid_y >= self.image_height
        ):
            return None
        return grid_x, self.image_height - 1 - grid_y

    def validate_goal(self, world_x, world_y):
        pixel = self.world_to_map_pixel(world_x, world_y)
        if pixel is None:
            return False, "目标在地图范围外"
        shade = float(self.occupancy_image.getpixel(pixel)) / 255.0
        occupancy = shade if self.negate else 1.0 - shade
        if occupancy >= self.occupied_thresh:
            return False, "目标位于障碍物上"
        if occupancy > self.free_thresh:
            return False, "目标位于未知区域"
        return True, ""

    def canvas_click(self, event):
        world_x, world_y = self.canvas_to_world(event.x, event.y)
        valid, reason = self.validate_goal(world_x, world_y)
        if not valid:
            self.selected_world = None
            self.goal_x_var.set("")
            self.goal_y_var.set("")
            self.draw_goal_marker()
            self.status_var.set(f"不可选择：{reason}")
            return
        self.selected_world = (world_x, world_y)
        self.goal_x_var.set(f"{world_x:.3f}")
        self.goal_y_var.set(f"{world_y:.3f}")
        self.draw_goal_marker()
        self.status_var.set(
            f"已选择 map 坐标 ({world_x:.3f}, {world_y:.3f})"
        )

    def draw_goal_marker(self):
        if self.goal_marker is not None:
            self.canvas.delete(self.goal_marker)
        if self.selected_world is None:
            return
        x, y = self.world_to_canvas(*self.selected_world)
        radius = 7
        self.goal_marker = self.canvas.create_oval(
            x - radius,
            y - radius,
            x + radius,
            y + radius,
            outline="#ff2020",
            fill="#ffff20",
            width=3,
        )

    def odom_callback(self, msg):
        self.robot_world = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )
        if self.robot_marker is not None:
            self.canvas.delete(self.robot_marker)
        x, y = self.world_to_canvas(*self.robot_world)
        radius = 6
        self.robot_marker = self.canvas.create_oval(
            x - radius,
            y - radius,
            x + radius,
            y + radius,
            outline="#0030ff",
            fill="#30d0ff",
            width=2,
        )

    def publish_selected_goal(self):
        try:
            goal_x = float(self.goal_x_var.get())
            goal_y = float(self.goal_y_var.get())
        except ValueError:
            messagebox.showerror("目标无效", "goal_x 和 goal_y 必须是数字")
            return
        if not math.isfinite(goal_x) or not math.isfinite(goal_y):
            messagebox.showerror("目标无效", "目标坐标必须是有限值")
            return
        valid, reason = self.validate_goal(goal_x, goal_y)
        if not valid:
            self.status_var.set(f"不可发布：{reason}")
            messagebox.showerror("目标无效", reason)
            return
        self.selected_world = (goal_x, goal_y)
        self.draw_goal_marker()
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.get_logger().info(
            f"Published next goal ({goal_x:.3f}, {goal_y:.3f})"
        )
        self.status_var.set(f"已发布目标 ({goal_x:.3f}, {goal_y:.3f})")
        self.root.withdraw()

    def event_callback(self, msg):
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if (
            isinstance(event, dict)
            and event.get("schema") == "semantic_nav_episode_event/v1"
            and event.get("event") == "ready"
        ):
            self.show_picker()

    def show_picker(self):
        self.root.deiconify()
        self.root.lift()
        self.root.attributes("-topmost", True)
        self.root.after(500, lambda: self.root.attributes("-topmost", False))
        self.status_var.set(
            str(self.get_parameter("ready_status_text").value)
        )


def main():
    rclpy.init()
    root = tk.Tk()
    node = EpisodeGoalPicker(root)

    def poll_ros():
        if not rclpy.ok():
            root.destroy()
            return
        rclpy.spin_once(node, timeout_sec=0.0)
        root.after(20, poll_ros)

    root.after(20, poll_ros)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
