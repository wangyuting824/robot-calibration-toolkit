"""Isaac Sim 6.1.0 public APIs, bootstrapped only when explicitly requested.

References: NVIDIA URDF migration, experimental Articulation, SimulationManager.
No substitute simulator is used when the SDK is unavailable.
"""
from pathlib import Path
import numpy as np
from core.io import InputError, number

_APPLICATIONS = []
TARGET_VERSION = "6.1.0"


def check_version(actual, expected=TARGET_VERSION):
    """Compare core versions; prerelease/build details remain in the report."""
    if expected not in (TARGET_VERSION, "5.1.0"):
        raise InputError(f"No documented adapter for requested Isaac Sim {expected}")
    if actual != expected:
        raise InputError(f"Isaac Sim version mismatch: expected {expected}, installed {actual}. Use the requested SDK launcher.")


def runtime_version(expected=TARGET_VERSION):
    # NVIDIA's version API reads the running application's VERSION file.
    from isaacsim.core.version import get_version
    parts = get_version()
    check_version(parts[0], expected)
    return {"isaac_sim_version": parts[0], "expected_isaac_sim_version": expected,
            "isaac_sim_build": list(parts)}


class _ApplicationSession:
    """Keep Kit alive until the worker has serialized its result."""
    def __init__(self, app):
        self.app = app

    def __getattr__(self, name):
        return getattr(self.app, name)

    def close(self):
        # The CLI worker owns process shutdown; eager close can terminate Kit
        # before result.json exists. All USD layers are saved by their producer.
        pass


def shutdown_applications():
    while _APPLICATIONS:
        _APPLICATIONS.pop().close()


def start_app(headless=True):
    try:
        from isaacsim import SimulationApp
    except ImportError as exc:
        raise InputError("Isaac Sim SDK unavailable. Run with Isaac Sim's Python launcher and install toolkit dependencies there.") from exc
    app = SimulationApp({"headless": headless, "fast_shutdown": True})
    _APPLICATIONS.append(app)
    return _ApplicationSession(app)


def numpy_value(value):
    return value.numpy() if hasattr(value, "numpy") else np.asarray(value)


class IsaacAdapter:
    capabilities = {"read_state": True, "command": True, "stop": True, "watchdog": False}

    def __init__(self, config):
        stage_path = Path(config.get("stage", "")).resolve()
        if not stage_path.is_file():
            raise InputError("Configure an existing Isaac USD stage")
        self.dt = number(config.get("physics_dt"))
        if self.dt <= 0:
            raise InputError("physics_dt must be positive")
        self.app = start_app(config.get("headless", True))
        try:
            version = runtime_version(config.get("expected_isaac_sim_version", TARGET_VERSION))
            import omni.usd
            import omni.timeline
            from isaacsim.core.simulation_manager import SimulationManager
            omni.usd.get_context().open_stage(str(stage_path))
            for _ in range(3):
                self.app.update()
            self.modern = version["isaac_sim_version"] == TARGET_VERSION
            if self.modern and not all(hasattr(SimulationManager, k) for k in ("setup_simulation", "step")):
                raise InputError("Isaac Sim 6.1.0 SimulationManager API is unavailable")
            from pxr import UsdPhysics
            stage = omni.usd.get_context().get_stage()
            scene_path = config.get("physics_scene_path", "/World/PhysicsScene")
            scene = UsdPhysics.Scene(stage.GetPrimAtPath(scene_path))
            if not scene:
                raise InputError("Select an existing PhysicsScene; no replacement scene is created")
            expected_gravity = np.asarray(scene.GetGravityDirectionAttr().Get()) * number(scene.GetGravityMagnitudeAttr().Get())
            self.rigid_view = None
            observed_names = config.get("observe_rigid_bodies", [])
            if observed_names:
                if not self.modern or len(set(observed_names)) != len(observed_names):
                    raise InputError("Unique rigid-body observations require the 6.1.0 adapter")
                from isaacsim.core.experimental.prims import RigidPrim
                body_paths = {}
                for prim in stage.Traverse():
                    if prim.HasAPI(UsdPhysics.RigidBodyAPI) and prim.GetName() in observed_names:
                        if prim.GetName() in body_paths:
                            raise InputError("Ambiguous observed rigid-body name")
                        body_paths[prim.GetName()] = str(prim.GetPath())
                if set(body_paths) != set(observed_names):
                    raise InputError("Missing observed rigid-body prim")
                # RigidPrim can author required PhysX APIs. Do this BEFORE the
                # simulation creates tensor views, otherwise edits invalidate them.
                self.rigid_view = RigidPrim(paths=[body_paths[n] for n in observed_names])
            if self.modern:
                from isaacsim.core.experimental.prims import Articulation
                self.manager = SimulationManager
                self.manager.setup_simulation(dt=self.dt, device=config.get("device", "cpu"))
                self.timeline = omni.timeline.get_timeline_interface()
                self.timeline.play()
                self.app.update()
                self.timeline.pause()
                self.robot = Articulation(paths=config["articulation_path"])
                self.names = list(self.robot.dof_names)
            else:
                from isaacsim.core.api import World
                from isaacsim.core.prims import SingleArticulation
                self.world = World(physics_dt=self.dt, rendering_dt=self.dt, stage_units_in_meters=1.0,
                                   physics_prim_path=scene_path, set_defaults=False)
                self.robot = self.world.scene.add(SingleArticulation(prim_path=config["articulation_path"], name="robotcal_test"))
                self.world.reset()
                self.names = list(self.robot.dof_names)
            self.time = 0.0
            actual_gravity = np.asarray(scene.GetGravityDirectionAttr().Get()) * number(scene.GetGravityMagnitudeAttr().Get())
            if not np.allclose(expected_gravity, actual_gravity, atol=1e-8):
                raise InputError("Simulation initialization changed the authored gravity")
            self.runtime_info = {**version, "api_family": "6.1.0 experimental" if self.modern else "5.1.0 legacy core",
                                 "physics_scene": scene_path, "physics_dt_s": self.dt,
                                 "gravity_m_s2": actual_gravity.tolist(), "dof_names": self.names}
            # Deliberate initial-state reset in simulation only; never used by real adapters.
            initial = config.get("initial_positions")
            if initial is not None:
                if set(initial) != set(self.names):
                    raise InputError("Simulation initial_positions must name every DOF")
                values = [number(initial[n]) for n in self.names]
                if self.modern:
                    self.robot.set_dof_positions([values])
                    self.robot.set_dof_velocities(np.zeros((1, len(self.names))))
                else:
                    self.robot.set_joint_positions(np.array(values))
                    self.robot.set_joint_velocities(np.zeros(len(self.names)))
                self.command(initial)
        except Exception:
            self.app.close()
            raise

    def command(self, target):
        if set(target) - set(self.names):
            raise InputError("Command contains a DOF absent from the imported articulation")
        indices = [self.names.index(n) for n in target]
        values = [number(target[n]) for n in target]
        if self.modern:
            self.robot.set_dof_position_targets([values], dof_indices=indices)
        else:
            from isaacsim.core.utils.types import ArticulationAction
            self.robot.apply_action(ArticulationAction(joint_positions=np.array(values), joint_indices=np.array(indices)))

    def advance(self, duration):
        count = round(duration / self.dt)
        if abs(count * self.dt - duration) > self.dt * 1e-6:
            raise InputError("Command time intervals must be integer multiples of physics_dt")
        if count:
            if self.modern:
                self.manager.step(steps=count)
            else:
                for _ in range(count):
                    self.world.step(render=False)
            self.time += count * self.dt

    def read_state(self):
        q = numpy_value(self.robot.get_dof_positions())[0] if self.modern else np.asarray(self.robot.get_joint_positions())
        dq = numpy_value(self.robot.get_dof_velocities())[0] if self.modern else np.asarray(self.robot.get_joint_velocities())
        return {"t": self.time, **{"q:" + n: float(v) for n, v in zip(self.names, q)},
                **{"dq:" + n: float(v) for n, v in zip(self.names, dq)}}

    def stop(self):
        if self.modern:
            self.timeline.stop()
        else:
            self.world.stop()

    def close(self):
        self.app.close()
