"""ProtoMotions agent and environment construction helpers for CrowdSim."""

from __future__ import annotations

import atexit
import faulthandler
import os
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from CrowdSim.scene_setup import CrowdRobotSceneConfig, resolve_repo_path


@dataclass
class ProtoMotionsRuntime:
    fabric: Any
    env: Any
    agent: Any
    robot_config: Any
    simulator_config: Any
    terrain_config: Any
    scene_lib_config: Any
    motion_lib_config: Any
    env_config: Any
    agent_config: Any
    simulation_app: Any
    app_launcher: Any | None = None


def enable_human_mesh(
    model_dir: str | None = None,
    hide_humanoid: bool = False,
    *,
    texture_dir: str | None = None,
    appearance_seed: int = 42,
    shape_std: float = 0.55,
    shape_clip: float = 1.25,
) -> None:
    os.environ["CROWDSIM_ENABLE_HUMAN_MESH"] = "1"
    os.environ["CROWDSIM_HIDE_HUMANOID"] = "1" if hide_humanoid else "0"
    os.environ["CROWDSIM_SMPL_MODEL_DIR"] = str(
        resolve_repo_path(model_dir or "data/smpl")
    )
    if texture_dir is None:
        smplitex_dir = resolve_repo_path("data/smpl/SMPLitex-textures")
        texture_dir = str(smplitex_dir) if smplitex_dir.is_dir() else (model_dir or "data/smpl")
    os.environ["CROWDSIM_SMPL_TEXTURE_DIR"] = str(resolve_repo_path(texture_dir))
    os.environ["CROWDSIM_HUMAN_APPEARANCE_SEED"] = str(int(appearance_seed))
    os.environ["CROWDSIM_SMPL_SHAPE_STD"] = str(float(shape_std))
    os.environ["CROWDSIM_SMPL_SHAPE_CLIP"] = str(float(shape_clip))


def resolve_robot_usd(value: str | None) -> str | None:
    if value is None:
        return None
    robot_name = value.lower()
    if robot_name == "jetbot":
        from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

        return f"{ISAAC_NUCLEUS_DIR}/Robots/NVIDIA/Jetbot/jetbot.usd"
    if robot_name in {"nova_carter", "novacarter", "nova-carter"}:
        local = resolve_repo_path("../Assets/Robots/NovaCarter/nova_carter.usd")
        if local.exists():
            return str(local)
        from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
        return f"{ISAAC_NUCLEUS_DIR}/Robots/NVIDIA/NovaCarter/nova_carter.usd"
    if "://" in value:
        return value

    path = Path(value).expanduser()
    resolved = path.resolve() if path.is_absolute() else resolve_repo_path(value)
    if not resolved.exists():
        raise FileNotFoundError(f"Robot USD not found: {resolved}")
    return str(resolved)


def make_crowd_robot_config(robot_cfg: dict, sensor_cfg: dict, robot_usd: str | None):
    if robot_usd is None:
        return None

    camera_cfg = sensor_cfg.get("camera", {})
    lidar_cfg = sensor_cfg.get("lidar", {})
    return CrowdRobotSceneConfig(
        usd_path=robot_usd,
        prim_name=robot_cfg.get("prim_name", "CrowdRobot"),
        init_z=float(robot_cfg.get("z", 0.0)),
        enable_camera=bool(camera_cfg.get("enabled", False)),
        camera_height=int(camera_cfg.get("height", 480)),
        camera_width=int(camera_cfg.get("width", 640)),
        camera_horizontal_fov=float(camera_cfg.get("horizontal_fov", 90.0)),
        camera_mount_prim_path=str(camera_cfg.get("mount_prim_path", "chassis_link")),
        camera_pos=tuple(float(v) for v in camera_cfg.get("pos", (0.25, 0.0, 0.6))),
        camera_rot=tuple(float(v) for v in camera_cfg.get("rot", (0.5, -0.5, 0.5, -0.5))),
        camera_convention=str(camera_cfg.get("convention", "ros")),
        enable_lidar=bool(lidar_cfg.get("enabled", False)),
        lidar_horizontal_res=float(lidar_cfg.get("horizontal_res", 1.0)),
        lidar_mesh_prim_paths=tuple(lidar_cfg.get("mesh_prims", ["/World/Scene"])),
        debug_vis=bool(lidar_cfg.get("debug_vis", False)),
    )


def configure_from_checkpoint(
    checkpoint: Path,
    motion_file: Path,
    num_envs: int,
    headless: bool,
    control_hz: float | None = None,
):
    import torch

    resolved_configs = torch.load(
        checkpoint.parent / "resolved_configs_inference.pt",
        map_location="cpu",
        weights_only=False,
    )
    robot_config = resolved_configs["robot"]
    simulator_config = resolved_configs["simulator"]
    terrain_config = resolved_configs.get("terrain")
    scene_lib_config = resolved_configs["scene_lib"]
    motion_lib_config = resolved_configs["motion_lib"]
    env_config = resolved_configs["env"]
    agent_config = resolved_configs["agent"]

    from protomotions.simulator.factory import update_simulator_config_for_test
    from protomotions.utils.inference_utils import apply_backward_compatibility_fixes

    current_simulator = simulator_config._target_.split(".")[-3]
    if current_simulator != "isaaclab":
        simulator_config = update_simulator_config_for_test(
            current_simulator_config=simulator_config,
            new_simulator="isaaclab",
            robot_config=robot_config,
        )

    apply_backward_compatibility_fixes(robot_config, simulator_config, env_config)
    if control_hz is not None:
        control_hz = float(control_hz)
        if control_hz <= 0.0:
            raise ValueError("control_hz must be positive")
        decimation = int(simulator_config.sim.decimation)
        simulator_config.sim.fps = int(round(control_hz * decimation))
        actual_hz = float(simulator_config.sim.fps) / decimation
        if abs(actual_hz - control_hz) > 1.0e-9:
            raise ValueError(
                f"Cannot represent control_hz={control_hz} with "
                f"decimation={decimation} and integer physics fps"
            )
        print(
            f"[CrowdSim] Runtime clock override: physics="
            f"{simulator_config.sim.fps}Hz decimation={decimation} "
            f"control={actual_hz:g}Hz dt={1.0 / actual_hz:.6f}s"
        )
    simulator_config.num_envs = int(num_envs)
    simulator_config.headless = bool(headless)
    if getattr(simulator_config, "projectile", None) is None:
        from protomotions.simulator.base_simulator.config import ProjectileConfig

        simulator_config.projectile = ProjectileConfig(num_projectiles=0)
    else:
        simulator_config.projectile.num_projectiles = 0
    print("[CrowdSim] ProtoMotions projectiles disabled (num_projectiles=0).")
    motion_lib_config.motion_file = str(motion_file)

    return (
        robot_config,
        simulator_config,
        terrain_config,
        scene_lib_config,
        motion_lib_config,
        env_config,
        agent_config,
    )


def create_fabric():
    from lightning.fabric import Fabric
    from protomotions.utils.fabric_config import FabricConfig

    fabric = Fabric(
        **asdict(
            FabricConfig(accelerator="gpu", devices=1, num_nodes=1, loggers=[], callbacks=[])
        )
    )
    fabric.launch()
    return fabric


def build_runtime(
    checkpoint: Path,
    motion_file: Path,
    num_envs: int,
    headless: bool,
    simulation_app,
    fabric,
    app_launcher=None,
    control_hz: float | None = None,
) -> ProtoMotionsRuntime:
    from protomotions.simulator.base_simulator.utils import convert_friction_for_simulator
    from protomotions.utils.component_builder import build_all_components
    from protomotions.utils.hydra_replacement import get_class

    (
        robot_config,
        simulator_config,
        terrain_config,
        scene_lib_config,
        motion_lib_config,
        env_config,
        agent_config,
    ) = configure_from_checkpoint(
        checkpoint, motion_file, num_envs, headless, control_hz=control_hz
    )

    terrain_config, simulator_config = convert_friction_for_simulator(
        terrain_config, simulator_config
    )
    components = build_all_components(
        terrain_config=terrain_config,
        scene_lib_config=scene_lib_config,
        motion_lib_config=motion_lib_config,
        simulator_config=simulator_config,
        robot_config=robot_config,
        device=fabric.device,
        save_dir=getattr(env_config, "save_dir", None),
        simulation_app=simulation_app,
    )

    env_cls = get_class(env_config._target_)
    env = env_cls(
        config=env_config,
        robot_config=robot_config,
        device=fabric.device,
        terrain=components["terrain"],
        scene_lib=components["scene_lib"],
        motion_lib=components["motion_lib"],
        simulator=components["simulator"],
    )

    agent_cls = get_class(agent_config._target_)
    agent = agent_cls(config=agent_config, env=env, fabric=fabric, root_dir=checkpoint.parent)
    agent.setup()
    agent.load(str(checkpoint), load_env=False)

    return ProtoMotionsRuntime(
        fabric=fabric,
        env=env,
        agent=agent,
        robot_config=robot_config,
        simulator_config=simulator_config,
        terrain_config=terrain_config,
        scene_lib_config=scene_lib_config,
        motion_lib_config=motion_lib_config,
        env_config=env_config,
        agent_config=agent_config,
        simulation_app=simulation_app,
        app_launcher=app_launcher,
    )


def shutdown_runtime(
    runtime: ProtoMotionsRuntime | None,
    nav_manager=None,
    *,
    traceback_after: float = 60.0,
) -> None:
    """Release a CrowdSim Isaac runtime in the installed-version-safe order.

    ProtoMotions' ``env.close()`` immediately calls ``SimulationApp.close()``.
    With Isaac Lab 2.3.2 that leaves the live ``SimulationContext`` and its
    callbacks in place while Kit closes the stage, which is the shutdown hang
    observed by the original PPO smoke.  Release project file handles and the
    context first, then ask Isaac Sim 5.1 for a normal (non-skip) cleanup.
    """
    if runtime is None:
        return

    faulthandler.enable()
    faulthandler.dump_traceback_later(float(traceback_after), repeat=True)
    print("[shutdown] begin", flush=True)

    if nav_manager is not None:
        close_logs = getattr(nav_manager, "_close_trajectory_logs", None)
        if callable(close_logs):
            close_logs()
        print("[shutdown] navigation logs closed", flush=True)

    env = getattr(runtime, "env", None)
    simulator = getattr(env, "simulator", None)
    sim = getattr(simulator, "_sim", None)
    if simulator is not None:
        # Base Simulator.close() only changes this flag.  Do that directly;
        # IsaacLabSimulator.close() must not run because it closes the app too
        # early, before the context teardown below.
        simulator._simulation_running = False

    if sim is not None:
        print("[shutdown] disable stop callback", flush=True)
        if hasattr(sim, "_disable_app_control_on_stop_handle"):
            sim._disable_app_control_on_stop_handle = True
        print("[shutdown] clear callbacks", flush=True)
        sim.clear_all_callbacks()
        print("[shutdown] clear SimulationContext", flush=True)
        type(sim).clear_instance()
    else:
        print("[shutdown] no SimulationContext instance", flush=True)

    simulation_app = getattr(runtime, "simulation_app", None)
    if simulation_app is None and simulator is not None:
        simulation_app = getattr(simulator, "_simulation_app", None)
    if simulation_app is None:
        faulthandler.cancel_dump_traceback_later()
        raise RuntimeError("CrowdSim runtime does not own a SimulationApp")

    # Isaac Sim's framework shutdown exits the process before close() returns
    # in the current 5.1 build.  atexit provides a final marker when CPython's
    # normal exit handlers are reached; the parent runner still records the
    # authoritative exit code and process lifetime.
    atexit.register(lambda: print("[shutdown] python exiting", flush=True))
    print(f"[shutdown] close_started_unix={time.time():.6f}", flush=True)
    print("[shutdown] close SimulationApp begin", flush=True)
    simulation_app.close(wait_for_replicator=False, skip_cleanup=False)
    print("[shutdown] close SimulationApp done", flush=True)
    faulthandler.cancel_dump_traceback_later()


def configure_viewer_camera(env, viewer_cfg: dict, headless: bool) -> None:
    if headless or viewer_cfg.get("camera_mode", "free") != "free":
        return

    import types
    import numpy as np

    eye = np.asarray(viewer_cfg.get("camera_eye", [8.0, -8.0, 6.0]), dtype=np.float64)
    target = np.asarray(viewer_cfg.get("camera_target", [0.0, 0.0, 1.0]), dtype=np.float64)

    def init_free_camera(self) -> None:
        self._cam_prev_char_pos = target.copy()
        self._perspective_view.set_camera_view(eye, target)

    def keep_user_camera(self) -> None:
        return

    env.simulator._init_camera = types.MethodType(init_free_camera, env.simulator)
    env.simulator._update_camera = types.MethodType(keep_user_camera, env.simulator)


def suppress_known_isaaclab_warning_spam() -> None:
    import omni.log

    omni_log = omni.log.get_log()
    for channel in ("omni.physx.plugin", "isaaclab.sim.utils", "omni.usd",
                     "carb.omniclient.plugin", "carb.windowing-glfw.plugin", "carb"):
        omni_log.set_channel_enabled(
            channel,
            False,
            omni.log.SettingBehavior.OVERRIDE,
        )
