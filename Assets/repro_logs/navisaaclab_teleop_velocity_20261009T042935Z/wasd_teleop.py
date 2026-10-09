#!/usr/bin/env python3
"""One CrowdSim robot, manually driven through the repository's own drive path."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import random
import select
import sys
import termios
import time
import tty

sys.dont_write_bytecode = True

SOURCE = Path("/home/user/workspace/NavIsaaclab2.0/Assets/repro_logs/navisaaclab_tilt_motion_20261008T173940Z/source").resolve()


class Keys:
    def __init__(self, enabled: bool):
        self.enabled = enabled
        self.fd = sys.stdin.fileno() if enabled else -1
        self.saved = None

    def __enter__(self):
        if self.enabled:
            if not sys.stdin.isatty():
                raise RuntimeError("INTERACTIVE_MODE_REQUIRES_A_TERMINAL")
            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, *_):
        if self.saved is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    def poll(self) -> str:
        if not self.enabled:
            return ""
        chunks = []
        while select.select([self.fd], [], [], 0)[0]:
            chunks.append(os.read(self.fd, 64).decode("utf-8", errors="ignore"))
        return "".join(chunks)


def camera_on_robot(env, xy) -> None:
    x, y = float(xy[0]), float(xy[1])
    env.simulator._sim.set_camera_view(
        eye=(x + 3.5, y - 4.5, 4.5),
        target=(x, y, 0.45),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--people", type=int, choices=(0, 1, 10), default=0)
    parser.add_argument("--show-markers", action="store_true")
    parser.add_argument("--speed", type=float, default=0.35, help="Forward m/s (no reverse in the current drive)")
    parser.add_argument("--turn", type=float, default=0.65, help="Turn rad/s")
    parser.add_argument("--pulse", type=float, default=0.8, help="Simulation seconds per keypress")
    parser.add_argument("--max-steps", type=int, default=0, help="0 runs until X, Ctrl+C, or GUI close")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--smoke", action="store_true", help="Bounded automatic input for runtime validation")
    args = parser.parse_args()
    if Path.cwd().resolve() != SOURCE:
        raise RuntimeError("RUN_FROM_FROZEN_SOURCE_ROOT")
    if not (0.0 < args.speed <= 1.0 and 0.0 < args.turn <= 1.0 and 0.1 <= args.pulse <= 3.0):
        raise ValueError("SPEED_TURN_OR_PULSE_OUT_OF_RANGE")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)

    # This import starts Isaac's required simulator shim before importing Torch.
    from CrowdSim.crowd_sim import cfg_path, load_config
    from CrowdSim.world.builder import build_env
    from CrowdSim.protomotions_runtime import shutdown_runtime
    import numpy as np
    import torch

    random.seed(703)
    np.random.seed(703)
    torch.manual_seed(703)
    torch.cuda.manual_seed_all(703)
    config = load_config(cfg_path("CrowdSim/config/env.yaml"))
    config["car"]["usd"] = "/home/user/isaacsim_assets/Assets/Isaac/5.1/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
    config["humanoid"]["checkpoint"] = "/home/user/workspace/NavIsaaclab2.0/NavIsaaclab2.0-main/data/pretrained_models/masked_mimic/smpl_57f98a9/last.ckpt"
    config["humanoid"]["motion_file"] = "/home/user/workspace/NavIsaaclab2.0/Assets/motion/amass_smpl_validation.pt"
    scene_cfg = load_config(cfg_path("CrowdSim/config/scenes/warehouse.yaml"))
    scene_cfg["scene_usd"] = "/home/user/isaacsim_assets/Assets/Isaac/5.1/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
    scene_cfg["scene_map"] = str(SOURCE / "CrowdSim/maps/newmap.yaml")
    config["scene"] = scene_cfg
    if config["scene"] != "warehouse":
        raise RuntimeError("EXPECTED_WAREHOUSE")
    config["navigation"]["num_humanoids"] = args.people
    config["navigation"]["num_robots"] = 1
    config["navigation"]["path"]["seed"] = 703
    config["navigation"]["rl"]["goal_curriculum"]["enabled"] = False
    config["navigation"]["recording"]["enabled"] = False
    # Path snapshots are written even when trajectory recording is disabled.
    config["navigation"]["recording"]["output_dir"] = str(output / "navigation")
    config["markers"]["enabled"] = args.show_markers
    config["humanoid"]["human_mesh"] = args.people > 0
    config["humanoid"]["state_recording"]["auto_record"] = False
    config["sensors"]["camera"]["auto_record"] = False
    (output / "config.json").write_text(json.dumps(config, indent=2) + "\n")

    runtime = navigation = None
    try:
        result = build_env(config, num_envs=max(args.people, 1), headless=args.headless)
        if result is None:
            raise RuntimeError("BUILD_ENV_RETURNED_NONE")
        env, agent, navigation, runtime = result
        assert navigation.config.num_robots == 1
        assert navigation.config.num_humanoids == args.people
        assert navigation.drive is not None
        agent.eval()
        app = env.simulator._simulation_app
        observations, _ = env.reset()
        initial_xy = navigation.drive.positions_xy()[0].copy()
        initial_yaw = float(navigation.drive.yaws()[0])
        if not args.headless:
            camera_on_robot(env, initial_xy)
        dt = float(navigation._env_dt)
        pulse_steps = max(1, int(round(args.pulse / dt)))
        print(f"TELEOP_READY robot=1 people={args.people} drive=KINEMATIC_ROOT_POSE "
              f"wheel_targets=ZERO start_xy={initial_xy.tolist()} yaw={initial_yaw:.3f}", flush=True)
        if not args.smoke:
            print("Terminal keys: W forward, A/D rotate, Q/E forward arcs, S or Space stop, "
                  "V recenter camera, X quit. Keep this terminal focused; each key grants "
                  f"{args.pulse:g} seconds of motion at at most 25 control steps/s. No reverse.", flush=True)

        motion = (0.0, 0.0)
        expires = 0
        step = 0
        max_executed_speed = 0.0
        wall_blocks = 0
        next_tick = time.monotonic()
        script = ([(0.0, 0.0)] * 5 + [(args.speed, 0.0)] * 25
                  + [(0.0, 0.0)] * 5 + [(0.0, args.turn)] * 25
                  + [(0.0, 0.0)] * 5) if args.smoke else None
        with Keys(enabled=not args.smoke) as keys, (output / "telemetry.csv").open("x", newline="") as trace, (output / "keys.jsonl").open("x") as events:
            writer = csv.writer(trace)
            writer.writerow(("step", "sim_seconds", "requested_v_mps", "requested_w_radps",
                             "x_m", "y_m", "yaw_rad", "executed_speed_mps", "wall_blocked",
                             "task_collision", "task_reached"))
            while app.is_running() and (args.max_steps == 0 or step < args.max_steps):
                if script is not None:
                    if step >= len(script):
                        break
                    motion = script[step]
                else:
                    exit_requested = False
                    for key in keys.poll().lower():
                        if key == "x":
                            exit_requested = True
                            break
                        if key in ("s", " "):
                            motion = (0.0, 0.0)
                            expires = step
                        elif key == "w":
                            motion = (args.speed, 0.0)
                            expires = step + pulse_steps
                        elif key == "a":
                            motion = (0.0, args.turn)
                            expires = step + pulse_steps
                        elif key == "d":
                            motion = (0.0, -args.turn)
                            expires = step + pulse_steps
                        elif key == "q":
                            motion = (args.speed, args.turn)
                            expires = step + pulse_steps
                        elif key == "e":
                            motion = (args.speed, -args.turn)
                            expires = step + pulse_steps
                        elif key == "v" and not args.headless:
                            camera_on_robot(env, navigation.drive.positions_xy()[0])
                        else:
                            continue
                        events.write(json.dumps({"step": step, "key": key,
                                                 "requested_v_mps": motion[0],
                                                 "requested_w_radps": motion[1]}) + "\n")
                        events.flush()
                        print(f"key={key!r} v={motion[0]:.2f} m/s "
                              f"omega={motion[1]:+.2f} rad/s", flush=True)
                    if exit_requested:
                        break
                    if step >= expires:
                        motion = (0.0, 0.0)

                command = np.array([[motion[0] / navigation.config.rl_max_linear_velocity,
                                     motion[1] / navigation.config.rl_max_angular_velocity]],
                                   dtype=np.float32)
                navigation.set_robot_rl_actions(command)
                obs = agent.add_agent_info_to_obs(observations)
                with torch.no_grad():
                    result = agent.model(agent.obs_dict_to_tensordict(obs))
                    human_action = result.get("mean_action", result["action"])
                _, _, human_done, _, _ = env.step(human_action)
                _, _, _, info, *_ = navigation.get_robot_rl_feedback()
                xy = navigation.drive.positions_xy()[0]
                yaw = float(navigation.drive.yaws()[0])
                executed_speed = float(np.linalg.norm(navigation.drive.velocities_xy()[0]))
                blocked = bool(navigation.drive.wall_blocked[0])
                collision = bool(info["collision"][0])
                reached = bool(info["reached"][0])
                max_executed_speed = max(max_executed_speed, executed_speed)
                wall_blocks += int(blocked)
                writer.writerow((step, round((step + 1) * dt, 4), *motion,
                                 float(xy[0]), float(xy[1]), yaw,
                                 executed_speed, int(blocked), int(collision), int(reached)))
                trace.flush()
                if step % 25 == 0 or blocked:
                    print(f"step={step} xy=({xy[0]:.2f},{xy[1]:.2f}) "
                          f"yaw={yaw:+.2f} executed={executed_speed:.2f} "
                          f"wall_blocked={int(blocked)}", flush=True)
                observations, _ = env.reset(human_done.nonzero(as_tuple=False).squeeze(-1))
                step += 1
                if not args.smoke:
                    next_tick += dt
                    now = time.monotonic()
                    if next_tick > now:
                        time.sleep(next_tick - now)
                    else:
                        next_tick = now
        if args.smoke:
            final_xy = navigation.drive.positions_xy()[0]
            final_yaw = float(navigation.drive.yaws()[0])
            yaw_change = math.atan2(math.sin(final_yaw - initial_yaw),
                                    math.cos(final_yaw - initial_yaw))
            summary = {"steps": step, "start_xy": initial_xy.tolist(),
                       "final_xy": final_xy.tolist(), "distance_m": float(np.linalg.norm(final_xy - initial_xy)),
                       "yaw_change_rad": yaw_change, "max_executed_speed_mps": max_executed_speed,
                       "wall_blocked_steps": wall_blocks}
            (output / "SMOKE_RESULT.json").write_text(json.dumps(summary, indent=2) + "\n")
            print("SMOKE_RESULT " + json.dumps(summary), flush=True)
            if step != len(script) or max_executed_speed < 0.1 or abs(yaw_change) < 0.25:
                raise RuntimeError("SMOKE_MOTION_CHECK_FAILED")
        (output / "COMPLETE.txt").write_text("TELEOP_COMPLETE\n")
    except KeyboardInterrupt:
        print("Ctrl+C: stopping teleoperation.", flush=True)
    finally:
        if runtime is not None:
            try:
                navigation.set_robot_rl_actions(np.zeros((1, 2), dtype=np.float32))
            except Exception:
                pass
            shutdown_runtime(runtime, navigation)


if __name__ == "__main__":
    main()
