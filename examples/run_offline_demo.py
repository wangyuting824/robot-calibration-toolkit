"""Run real toolkit commands against SYNTHETIC data and exercise delivery gates."""
from pathlib import Path
import argparse
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from examples.generate_demo import generate
from core.io import save


def run_demo(destination):
    data = generate(destination)
    output = data / "results"
    output.mkdir(exist_ok=True)

    def run(command, *arguments, expected=0):
        invocation = [sys.executable, str(ROOT / "run.py"), command, *map(str, arguments), "--output", str(output / (command + ".json"))]
        executed = subprocess.run(invocation, cwd=ROOT, check=False)
        if executed.returncode != expected:
            raise RuntimeError(f"Unexpected exit code {executed.returncode}: {invocation}")

    run("preflight")
    run("validate-structure", "--parameters", data / "parameters.xlsx", "--require-inertia")
    run("build-urdf", "--parameters", data / "parameters.xlsx", "--baseline", data / "nominal.urdf", "--destination", data / "model", "--mechanisms", data / "mechanisms.yaml")
    run("calibrate-joint-reference", "--data", data / "joint_reference.csv", "--manifest", data / "joint_reference.yaml", "--f-scale", 0.001)
    run("verify-joint-limits", "--data", data / "limits.csv", "--manifest", data / "limits_manifest.yaml", "--specification", data / "limits_spec.yaml")
    run("calibrate-kinematics", "--data", data / "kinematics_calibration.csv", "--manifest", data / "kinematics_calibration.yaml", "--urdf", data / "nominal.urdf", "--config", data / "calibration.yaml")
    run("validate-kinematics", "--data", data / "kinematics_validation.csv", "--manifest", data / "kinematics_validation.yaml", "--urdf", data / "nominal.urdf", "--calibration", output / "calibrate-kinematics.json", "--acceptance", data / "acceptance.yaml", "--errors-csv", data / "kinematics_errors.csv")
    run("identify-dynamics", "--data", data / "dynamics_fit.csv", "--manifest", data / "dynamics_fit.yaml", "--urdf", data / "nominal.urdf", "--config", data / "dynamics_config.yaml")
    run("tune-drives", "--data", data / "drives.csv", "--manifest", data / "drives.yaml")
    run("identify-contact", "--data", data / "contact.csv", "--manifest", data / "contact.yaml", "--config", data / "contact_config.yaml")
    run("validate-dynamics", "--real-data", data / "response_real.csv", "--real-manifest", data / "response_real.yaml", "--sim-data", data / "response_sim.csv", "--sim-manifest", data / "response_sim.yaml", "--acceptance", data / "acceptance.yaml", "--errors-csv", data / "response_errors.csv")
    run("capture-data", "--interface", data / "replay_interface.yaml", "--manifest", data / "replay_manifest.yaml", "--output-data", data / "capture.csv", "--output-manifest", data / "capture.yaml")
    run("run-test", "--target", "replay", "--interface", data / "replay_interface.yaml", "--output-data", data / "replayed.csv")
    run("generate-report", "--results", output, "--destination", data / "report")
    config = {"stage": "kinematics", "reports": [str(p) for p in output.glob("*.json")], "artifacts": [
        {"delivery": "D01", "path": str(data / "parameters.xlsx")},
        {"delivery": "D02", "path": str(data / "model")},
        {"delivery": "D03", "path": str(data / "D03_readiness.yaml")},
        {"delivery": "D04", "path": str(ROOT / "examples")},
        {"delivery": "D05", "path": str(data / "report")},
        {"delivery": "D06", "path": str(ROOT / "docs")}]}
    save(data / "delivery.yaml", config)
    run("package-delivery", "--config", data / "delivery.yaml", "--destination", data / "synthetic-draft.zip")
    run("package-delivery", "--config", data / "delivery.yaml", "--destination", data / "must-not-exist.zip", "--final", expected=2)
    print("Synthetic offline demo completed. No real robot or Isaac physics acceptance is claimed.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--destination", required=True)
    run_demo(parser.parse_args().destination)
