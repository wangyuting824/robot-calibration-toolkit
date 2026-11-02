"""Optional REAL Isaac SDK test, using only the synthetic reference arm.

Each SimulationApp runs in its own process. No real robot adapter is loaded.
"""
from pathlib import Path
import argparse
import os
import subprocess
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from examples.generate_demo import generate
from core.io import load, save, write_csv, csv_rows, number


def run_smoke(destination, launcher, extra_pythonpath=None, expected_version="6.1.0"):
    data = generate(destination)
    output = data / "results"
    output.mkdir(exist_ok=True)
    env = dict(os.environ)
    for key in ("CONDA_PREFIX", "ISAAC_PATH", "EXP_PATH", "CARB_APP_PATH"):
        env.pop(key, None)
    env["PYTHONPATH"] = os.pathsep.join(str(p) for p in [ROOT, extra_pythonpath] if p)

    def run(command, *args):
        target = output / (command + ".json")
        completed = subprocess.run([str(launcher), str(ROOT / "run.py"), command, *map(str, args), "--output", str(target)],
                                   cwd=ROOT, env=env, check=False)
        if completed.returncode:
            raise RuntimeError(f"{command} failed: inspect {target}")
        return load(target)

    run("build-urdf", "--parameters", data / "parameters.xlsx", "--baseline", data / "nominal.urdf",
        "--mechanisms", data / "mechanisms.yaml", "--destination", data / "model", "--require-inertia")
    config = {"expected_isaac_sim_version": expected_version,
              "fix_base": True, "headless": True, "physics_backend": "physx", "physics_dt": 1 / 240,
              "gravity": [0, 0, 0], "self_collision": False,
              "drives": {n: {"type": "force", "stiffness": 20, "damping": 2, "max_effort": 100} for n in ("j1", "j2")}}
    save(data / "isaac.yaml", config)
    built = run("build-isaac-asset", "--urdf", data / "model/robot.urdf", "--config", data / "isaac.yaml", "--destination", data / "isaac")
    thresholds = load(data / "acceptance.yaml")
    thresholds["asset"].update(axis_tolerance_rad=1e-5, anchor_tolerance_m=1e-5,
                                mass_tolerance_kg=1e-5, com_tolerance_m=1e-5, inertia_tolerance_kg_m2=1e-5)
    save(data / "acceptance.yaml", thresholds)
    run("validate-isaac-asset", "--urdf", data / "model/robot.urdf", "--stage", built["scene"], "--acceptance", data / "acceptance.yaml")
    roots = built["articulation_paths"]
    if len(roots) != 1:
        raise RuntimeError(f"Select a single articulation root: {roots}")
    save(data / "isaac_interface.yaml", {"class": "adapters.isaac_adapter:IsaacAdapter", "start_tolerance_rad_or_m": 1e-5,
        "options": {"expected_isaac_sim_version": expected_version,
                    "stage": built["scene"], "articulation_path": roots[0], "physics_dt": 1 / 240,
                    "headless": True, "device": "cpu", "initial_positions": {"j1": 0, "j2": 0}}})
    trajectory = [{"t": float(t), "q:j1": float(0.04 * np.sin(np.pi * t)), "q:j2": float(-0.03 * np.sin(np.pi * t))}
                  for t in np.linspace(0, 1, 61)]
    write_csv(data / "trajectory.csv", trajectory)
    measured = run("run-test", "--target", "sim", "--interface", data / "isaac_interface.yaml", "--urdf", data / "model/robot.urdf",
        "--trajectory", data / "trajectory.csv", "--output-data", data / "isaac_response.csv")
    rows = csv_rows(data / "isaac_response.csv")
    times = np.array([number(r["t"]) for r in rows])
    q1 = np.array([number(r["q:j1"]) for r in rows])
    q2 = np.array([number(r["q:j2"]) for r in rows])
    if len(rows) != len(trajectory) or not np.all(np.diff(times) > 0) or max(q1) <= 1e-6 or min(q2) >= -1e-6:
        raise RuntimeError("SDK returned no valid timed motion in the commanded directions")
    if not np.allclose(measured["runtime_info"]["gravity_m_s2"], [0, 0, 0]):
        raise RuntimeError("SDK did not retain the synthetic test's authored zero gravity")
    save(data / "sdk_smoke_summary.json", {"status": "sdk_smoke_pass", "data_kind": "synthetic",
        "samples": len(rows), "j1_range": [min(q1), max(q1)], "j2_range": [min(q2), max(q2)],
        "runtime_info": measured["runtime_info"], "is_real_robot_acceptance": False})
    print("Isaac SDK smoke test completed on a synthetic arm; no real robot precision is asserted.")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--destination", required=True)
    p.add_argument("--launcher", required=True)
    p.add_argument("--extra-pythonpath")
    p.add_argument("--expected-version", choices=["6.1.0", "5.1.0"], default="6.1.0",
                   help="5.1.0 is an explicit legacy regression test, never a 6.1.0 validation")
    args = p.parse_args()
    run_smoke(args.destination, args.launcher, args.extra_pythonpath, args.expected_version)
