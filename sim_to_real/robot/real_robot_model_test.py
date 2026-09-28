#!/usr/bin/env python3
"""Run VLA, SemanticCNN, or DRL-VO through the same robot interface."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("model", choices=("vla", "semantic_cnn", "drl_vo"))
    parser.add_argument(
        "launch_args",
        nargs="*",
        help="optional ROS launch arguments, for example goal_x:=-2.0",
    )
    args = parser.parse_args()

    robot_root = Path(__file__).resolve().parent
    if args.model == "vla":
        if args.launch_args:
            parser.error("the unchanged VLA entry does not accept ROS launch arguments")
        os.chdir(robot_root)
        os.execv(
            sys.executable,
            [sys.executable, str(robot_root / "real_robot_infer_node3_clip.py")],
        )

    model_root = robot_root / "comparison_models"
    ros_setup = Path("/opt/ros") / os.environ.get("ROS_DISTRO", "humble") / "setup.bash"
    install_setup = model_root / "ros2_ws/install/setup.bash"
    if not ros_setup.is_file():
        parser.error(f"ROS setup not found: {ros_setup}")
    if not install_setup.is_file():
        try:
            subprocess.run(
                [str(model_root / "build.sh")],
                cwd=robot_root,
                check=True,
            )
        except subprocess.CalledProcessError as error:
            parser.error(f"comparison model build failed with code {error.returncode}")

    environment = os.environ.copy()
    environment["ROBOT_COMPARISON_MODELS_ROOT"] = str(model_root)
    environment["SIM_TO_REAL_BUNDLE_ROOT"] = str(model_root)
    third_party = str(model_root / "third_party/dr_spaam")
    environment["PYTHONPATH"] = third_party + (
        ":" + environment["PYTHONPATH"] if environment.get("PYTHONPATH") else ""
    )

    shell = r'''
set -e
source "$1"
source "$2"
shift 2
exec "$@"
'''
    command = [
        "/bin/bash",
        "-c",
        shell,
        "robot-model-test",
        str(ros_setup),
        str(install_setup),
        "ros2",
        "launch",
        "semantic_nav_runtime",
        "robot_model_test.launch.py",
        f"method:={args.model}",
        *args.launch_args,
    ]
    os.execvpe(command[0], command, environment)


if __name__ == "__main__":
    main()
