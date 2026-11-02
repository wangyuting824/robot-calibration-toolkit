"""Invoke NVIDIA's own headless system-identification tool, without reimplementing it.

Official backend is Beta and version dependent. The installed official tool owns
telemetry validation, recipe interpretation, optimization and rollout checks.
"""
from pathlib import Path
import subprocess
from core.io import InputError, load, result, sha256


def run(args):
    config = load(args.config)
    expected = config.get("expected_isaac_sim_version", "6.1.0")
    if expected != "6.1.0":
        raise InputError("This SysID wrapper targets the documented NVIDIA 6.1.0 interface")
    keys = ("python_launcher", "official_tool", "stage", "telemetry", "recipe", "robot_prim")
    if any(not config.get(k) for k in keys):
        raise InputError(f"Configure all official SysID inputs: {keys}")
    for key in ("python_launcher", "official_tool", "stage", "telemetry"):
        if not Path(config[key]).is_file():
            raise InputError(f"Missing {key}: {config[key]}")
        config[key] = str(Path(config[key]).resolve())
    tool = Path(config["official_tool"])
    if tool.name != "headless_sysid_solve.py":
        raise InputError("Point official_tool to NVIDIA extension tools/headless_sysid_solve.py")
    version_file = tool.resolve().parents[3] / "VERSION"
    if not version_file.is_file():
        raise InputError("Cannot verify SDK VERSION from the installed extension tool path")
    installed = version_file.read_text().strip()
    if installed.split("-", 1)[0].split("+", 1)[0] != expected:
        raise InputError(f"Official tool SDK version mismatch: expected {expected}, installed {installed}")
    evidence = {"isaac_sim_build": installed, "official_tool_sha256": sha256(tool)}
    command = [str(config["python_launcher"]), str(tool), "--robot", config["robot_prim"],
               "--telemetry", str(config["telemetry"]), "--recipe", config["recipe"], "--stage", str(config["stage"])]
    if not args.execute:
        return result("run-nvidia-sysid", {**evidence, "command_line": command, "executed": False,
                      "backend": "NVIDIA isaacsim.robot_setup.sysid (Beta)"}, [args.config], "planned")
    log_path = Path(args.log)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    timed_out = False
    exit_code = None
    with log_path.open("w", encoding="utf-8") as log:
        try:
            process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=False,
                                     cwd=log_path.parent, timeout=number_timeout(config.get("timeout_s", 3600)))
            exit_code = process.returncode
        except subprocess.TimeoutExpired:
            timed_out = True
    return result("run-nvidia-sysid", {**evidence, "command_line": command, "executed": True,
                  "exit_code": exit_code, "timed_out": timed_out, "log": str(log_path),
                  "qualification": "Inspect official pre-solve check and holdout results; tool exit alone is not robot acceptance"},
                  [args.config, tool, config["telemetry"], config["stage"]], "completed" if exit_code == 0 else "failed")


def number_timeout(value):
    from core.io import number
    value = number(value)
    if value <= 0:
        raise InputError("timeout_s must be positive")
    return value
