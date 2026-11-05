"""Launch the installed Inspect Robots CLI with per-episode history reset for this task.

    python tasks/occluded-service-block/run_task.py --policy yam_scripted --log-dir runs/cover-demo

The reset adapter is process-local; it does not edit yam_arena or depend on an undocumented reset hook.
"""

import argparse
import functools
import json
import sys
from importlib.metadata import entry_points
from pathlib import Path

TASK = Path(__file__).resolve().with_name("task.py")
TASK_ARGUMENT = Path("tasks/occluded-service-block/task.py")
INSTRUCTION = json.loads(TASK.with_suffix(".json").read_text(encoding="utf-8"))["instruction"]


def install_contact_sampling(embodiment):
    """Read each filtered fingertip/object pair after every physics scene update, before scoring.

    Singleton filters avoid depending on undocumented ordering of multiple partner bodies.
    The installed backend must expose normal_force_matrix_w; no distance-based fallback is allowed.
    """
    import numpy as np

    module = embodiment.task.module
    scene = embodiment._base.scene
    original_update = scene.update
    embodiment._contact_frames = []
    embodiment._capture_contacts = False
    mapping = {}
    for arm in module.ARMS:
        for tip in ("tip_left", "tip_right"):
            for name in module.CONTACT_OBJECTS:
                key = f"audit_{arm}_{tip}_{name}"
                sensor = scene[key]
                if sensor.body_names != [tip]:
                    raise RuntimeError(f"Contact source is not the expected single fingertip: {key}")
                mapping[key] = {"source": sensor.cfg.prim_path, "partner": sensor.cfg.filter_prim_paths_expr}
    print("contact_mapping: " + json.dumps(mapping, sort_keys=True))

    @functools.wraps(original_update)
    def update(dt, *args, **kwargs):
        original_update(dt, *args, **kwargs)
        if not embodiment._capture_contacts:
            return
        if not np.isclose(dt, 1 / module.PHYSICS_HZ):
            raise RuntimeError("Unexpected physics sampling interval")
        frame = {arm: {name: [] for name in module.CONTACT_OBJECTS} for arm in module.ARMS}
        for arm in module.ARMS:
            for tip in ("tip_left", "tip_right"):
                for name in module.CONTACT_OBJECTS:
                    sensor = scene[f"audit_{arm}_{tip}_{name}"]
                    matrix = sensor.data.normal_force_matrix_w
                    if matrix is None:
                        raise RuntimeError("Filtered normal-force telemetry is unavailable")
                    matrix = matrix.torch
                    if tuple(matrix.shape) != (1, 1, 1, 3):
                        raise RuntimeError("Contact filter must resolve one environment, fingertip and partner shape")
                    # Take the norm for EACH fingertip, so opposing normal forces cannot cancel.
                    frame[arm][name].append(float(matrix[0, 0, 0].norm().item()))
        embodiment._contact_frames.append(frame)
        if len(embodiment._contact_frames) > module.SUBSTEPS:
            raise RuntimeError("Contact sampling exceeded one control period")

    scene.update = update


def install_reset_adapter():
    from yam_arena.inspect_embodiment import YamArenaEmbodiment

    original_reset = YamArenaEmbodiment.reset
    original_close = YamArenaEmbodiment.close
    original_task_step = YamArenaEmbodiment._task_step
    original_ensure_sim = YamArenaEmbodiment._ensure_sim

    @functools.wraps(original_ensure_sim)
    def ensure_sim(self, goal):
        if self.task is None or self.task.path != TASK or self._env is not None:
            return original_ensure_sim(self, goal)
        # Start Kit before importing any Isaac-dependent scene code, as the upstream launcher does.
        import atexit
        import os

        from yam_arena import inspect_embodiment

        if inspect_embodiment._APP is None:
            from isaaclab.app import AppLauncher

            os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
            inspect_embodiment._APP = AppLauncher(
                headless=self.headless, device=self.device, enable_cameras=True
            ).app
            atexit.register(inspect_embodiment._close_app_at_exit)

        from isaaclab.sensors import ContactSensorCfg
        from yam_arena import scene as scene_module
        from yam_arena.robot import asset_link_paths

        original_callback = scene_module.env_cfg_callback
        links = asset_link_paths()
        module = self.task.module

        def callback(control_hz=None):
            configure = original_callback(control_hz)

            def configure_contacts(cfg):
                cfg = configure(cfg)
                for arm in module.ARMS:
                    robot = getattr(cfg.scene, f"{arm}_robot")
                    robot.spawn.activate_contact_sensors = True
                    for tip in ("tip_left", "tip_right"):
                        for name in module.CONTACT_OBJECTS:
                            partner = getattr(cfg.scene, name).prim_path
                            sensor = ContactSensorCfg(
                                prim_path=robot.prim_path + links[tip],
                                filter_prim_paths_expr=[partner], update_period=0.0,
                                history_length=module.SUBSTEPS + 2,
                            )
                            setattr(cfg.scene, f"audit_{arm}_{tip}_{name}", sensor)
                return cfg

            return configure_contacts

        scene_module.env_cfg_callback = callback
        try:
            original_ensure_sim(self, goal)
        finally:
            scene_module.env_cfg_callback = original_callback
        install_contact_sampling(self)

    @functools.wraps(original_reset)
    def reset(self, scene, *, seed=None):
        selected = self.task is not None and self.task.path == TASK
        if selected and (not 19.999 < self.info.control_hz < 20.001 or self.object_jitter != 0):
            raise ValueError("This fixed-layout task requires control_hz=20 and object_jitter=0")
        if selected and self._env is not None:
            self._capture_contacts = False
        observation = original_reset(self, scene, seed=seed)
        if selected:
            from yam_arena.scene import object_states

            self.task.module.reset_episode(object_states(self._env, self.task.names), self.info.control_hz)
            self._contact_frames.clear()
            self._capture_contacts = True
        return observation

    @functools.wraps(original_close)
    def close(self):
        if self.task is not None and self.task.path == TASK:
            print("task_audit: " + json.dumps(self.task.module.diagnostics(), sort_keys=True))
        return original_close(self)

    @functools.wraps(original_task_step)
    def task_step(self):
        if self.task is not None and self.task.path == TASK:
            import numpy as np

            from yam_arena.constants import FINGER_OPEN
            from yam_arena.runtime import tcp_world

            arms = {}
            for arm in ("left", "right"):
                key = f"{arm}_robot"
                robot = self._base.scene[key]
                finger_ids = self._ids[key][1]
                q = robot.data.joint_pos.torch[0, finger_ids]
                arms[arm] = {
                    "tcp": tcp_world(robot),
                    "opening": float(np.clip(q.mean().item() / FINGER_OPEN, 0.0, 1.0)),
                }
            frames, self._contact_frames = self._contact_frames, []
            contacts = {
                arm: {name: np.asarray([frame[arm][name] for frame in frames])
                      for name in self.task.module.CONTACT_OBJECTS}
                for arm in self.task.module.ARMS
            }
            self.task.module.observe_robot(arms, contacts)
        return original_task_step(self)

    YamArenaEmbodiment.reset = reset
    YamArenaEmbodiment.close = close
    YamArenaEmbodiment._task_step = task_step
    YamArenaEmbodiment._ensure_sim = ensure_sim


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", choices=("yam_scripted", "dream"), default="yam_scripted")
    parser.add_argument("--log-dir", required=True)
    args = parser.parse_args()
    if TASK_ARGUMENT.resolve() != TASK:
        parser.error("Run this launcher from the benchmark-tasks repository root")
    install_reset_adapter()
    sys.argv = [
        "inspect-robots",
        "run",
        "--embodiment",
        "yam_arena",
        "--policy",
        args.policy,
        "-E",
        f"task={TASK_ARGUMENT.as_posix()}",
        "-E",
        "object_jitter=0",
        "-E",
        "control_hz=20",
        "-E",
        "cam_width=320",
        "-E",
        "cam_height=176",
        "--instruction",
        INSTRUCTION,
        "--scorer",
        "task_score",
        "--max-steps",
        "2400",
        "--store-frames",
        "--log-dir",
        args.log_dir,
    ]
    cli = next(ep for ep in entry_points(group="console_scripts") if ep.name == "inspect-robots")
    cli.load()()


if __name__ == "__main__":
    main()
