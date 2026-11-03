"""View an already tested model; arrows select recorded/test poses, R replays.

Pure pose playback via mj_forward, not a dynamics or controller experiment.
No hardware, ROS, sockets or drive command interfaces are imported.
"""
import argparse
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from core.io import load, save
from core.model import Model
from core.mujoco_model import set_pose


def run(args):
    import mujoco
    import mujoco.viewer
    import numpy as np
    nominal = Model(args.urdf)
    model = mujoco.MjModel.from_xml_path(args.mjcf)
    data = mujoco.MjData(model)
    poses = load(args.poses)["poses"]
    state = {"index": 0, "replay": False}

    def key(code):
        if code in (262, 263):
            state["index"] = (state["index"] + (1 if code == 262 else -1)) % len(poses)
            state["replay"] = False
        elif code in (82, 114):
            state["replay"] = not state["replay"]

    set_pose(nominal, model, data, poses[0])
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.lookat[:] = model.stat.center
    camera.distance = max(1.5, float(model.stat.extent) * 1.25)
    camera.azimuth, camera.elevation = 130, -20
    option = mujoco.MjvOption()
    option.geomgroup[0] = False  # URDF collision group; visuals are group 1.
    if args.screenshot:
        from PIL import Image
        model.vis.global_.offwidth = 960
        model.vis.global_.offheight = 720
        with mujoco.Renderer(model, height=720, width=960) as renderer:
            renderer.update_scene(data, camera=camera, scene_option=option)
            pixels = renderer.render()
            if np.std(pixels) < 1:
                raise RuntimeError("Rendering produced an empty image")
            Image.fromarray(pixels).save(args.screenshot)
        print("Screenshot: " + args.screenshot, flush=True)
    if args.headless:
        return
    with mujoco.viewer.launch_passive(model, data, key_callback=key) as viewer:
        with viewer.lock():
            viewer.cam.lookat[:] = camera.lookat
            viewer.cam.distance = camera.distance
            viewer.cam.azimuth, viewer.cam.elevation = camera.azimuth, camera.elevation
            viewer.opt.geomgroup[0] = False
        if args.ready_file:
            save(args.ready_file, {"status": "viewer_open", "backend": "MuJoCo", "version": mujoco.__version__,
                "mode": "pose playback; no physics stepping", "hardware_control_commands_sent": False})
        print("MuJoCo viewer ready. Left/Right: select pose. R: replay. No hardware commands.", flush=True)
        started, changed = time.monotonic(), time.monotonic()
        while viewer.is_running() and (not args.duration_s or time.monotonic() - started < args.duration_s):
            if state["replay"] and time.monotonic() - changed > .25:
                state["index"] = (state["index"] + 1) % len(poses)
                changed = time.monotonic()
            with viewer.lock():
                set_pose(nominal, model, data, poses[state["index"]])
            viewer.sync()
            time.sleep(1 / 30)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("urdf", "mjcf", "poses"):
        p.add_argument("--" + name, required=True)
    p.add_argument("--screenshot")
    p.add_argument("--headless", action="store_true")
    p.add_argument("--ready-file")
    p.add_argument("--duration-s", type=float, default=0)
    run(p.parse_args())
