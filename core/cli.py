"""Unified executable interface; every command emits machine-readable evidence."""
import argparse
from datetime import datetime, timezone
from importlib import import_module
from pathlib import Path
import sys
import subprocess
import importlib.util

from core.io import InputError, result, save, load
from core.provenance import basis

COMMANDS = {
    "preflight": ("common.preflight", "检查软件、资料和 SDK 条件"),
    "check-openarm": ("common.check_openarm", "检查 OpenArm CAN 状态；可选非运动状态查询"),
    "prepare-inputs": ("common.prepare_inputs", "整理现有 ZIP 并生成未确认参数主表"),
    "validate-structure": ("stage_01_definition.validate_structure", "检查结构及参数定义"),
    "build-urdf": ("stage_01_definition.build_urdf", "生成 URDF 模型包"),
    "capture-data": ("common.capture_data", "通过明确配置的接口采集或导入数据"),
    "run-test": ("common.run_test", "回放数据或执行明确指定的仿真/真机测试"),
    "calibrate-joint-reference": ("stage_02_kinematics.calibrate_joint_reference", "拟合反馈换算与零偏"),
    "verify-joint-limits": ("stage_02_kinematics.verify_joint_limits", "核验记录的限位，不搜索机械止挡"),
    "calibrate-kinematics": ("stage_02_kinematics.calibrate_kinematics", "拟合选定运动学参数"),
    "validate-kinematics": ("stage_02_kinematics.validate_kinematics", "用独立数据验证位姿误差"),
    "build-isaac-asset": ("stage_03_isaac_dynamics.build_isaac_asset", "调用 NVIDIA Isaac Sim 6.1.0 导入 API"),
    "validate-isaac-asset": ("stage_03_isaac_dynamics.validate_isaac_asset", "检查 USD/URDF 静态一致性"),
    "validate-runtime-kinematics": ("stage_03_isaac_dynamics.validate_runtime_kinematics", "比较工具包 FK 与 Isaac 实际连杆位姿；仅仿真"),
    "run-nvidia-sysid": ("stage_03_isaac_dynamics.run_nvidia_sysid", "调用 NVIDIA 官方 System Identification 工具"),
    "identify-dynamics": ("stage_03_isaac_dynamics.identify_dynamics", "项目 RNEA/关节摩擦拟合"),
    "tune-drives": ("stage_03_isaac_dynamics.tune_drives", "从独立 PD 力矩分量拟合力驱动增益"),
    "identify-contact": ("stage_03_isaac_dynamics.identify_contact", "拟合显式指定的接触近似模型"),
    "validate-dynamics": ("stage_03_isaac_dynamics.validate_dynamics", "比较匹配工况下的真机与仿真数据"),
    "generate-report": ("common.generate_report", "汇总 HTML 与 JSON 报告"),
    "package-delivery": ("common.package_delivery", "检查并打包六类交付物"),
}


def parser():
    root = argparse.ArgumentParser(description="Robot calibration toolkit — project code with documented method sources")
    sub = root.add_subparsers(dest="command", required=True)
    for command, (_, description) in COMMANDS.items():
        p = sub.add_parser(command, help=description, description=description)
        p.add_argument("--output", help="Result JSON path; default is a unique file under outputs/")
        p.add_argument("--debug", action="store_true")
        p.add_argument("--sdk-worker", action="store_true", help=argparse.SUPPRESS)

        def option(name, **kwargs):
            p.add_argument("--" + name.replace("_", "-"), **kwargs)
        if command == "preflight":
            option("sources")
            option("require_isaac", action="store_true")
        elif command == "check-openarm":
            option("interfaces", nargs="+", default=["can0", "can1"])
            option("listen_seconds", type=float, default=2)
            option("query_state", action="store_true")
            option("probe_warning", action="store_true", help="Allow a bounded ID 1..8 diagnostic on ERROR-WARNING interfaces; abort on ERROR-PASSIVE/BUS-OFF")
            option("raw_frames", default="outputs/openarm/raw_frames.json")
        elif command == "prepare-inputs":
            option("archive", required=True)
            option("destination", default="inputs/baseline_model")
            option("workbook", default="templates/robot_parameters.xlsx")
        elif command == "validate-structure":
            option("parameters", required=True)
            option("mechanisms")
            option("require_inertia", action="store_true")
        elif command == "build-urdf":
            for name in ("parameters", "baseline", "destination"):
                option(name, required=True)
            option("calibration")
            option("mechanisms")
            option("require_inertia", action="store_true")
            option("draft", action="store_true")
        elif command == "capture-data":
            for name in ("interface", "manifest", "output_data", "output_manifest"):
                option(name, required=True)
            option("max_samples", type=int, default=10000)
        elif command == "run-test":
            option("target", required=True, choices=["replay", "sim", "real"])
            option("interface", required=True)
            option("output_data", required=True)
            option("trajectory")
            option("urdf")
            option("execute", action="store_true")
        elif command in ("calibrate-joint-reference", "verify-joint-limits", "calibrate-kinematics", "validate-kinematics", "identify-dynamics", "identify-contact", "tune-drives"):
            option("data", required=True)
            option("manifest", required=True)
            if command in ("calibrate-kinematics", "validate-kinematics", "identify-dynamics"):
                option("urdf", required=True)
            if command in ("calibrate-kinematics", "identify-dynamics", "identify-contact"):
                option("config", required=True)
            if command == "calibrate-joint-reference":
                option("f_scale", type=float, required=True, help="Declared robust loss scale, in reference position units")
            if command == "verify-joint-limits":
                option("specification", required=True)
            if command == "validate-kinematics":
                option("calibration", required=True)
                option("acceptance", required=True)
                option("errors_csv")
        elif command == "build-isaac-asset":
            for name in ("urdf", "config", "destination"):
                option(name, required=True)
        elif command == "validate-isaac-asset":
            for name in ("urdf", "stage", "acceptance"):
                option(name, required=True)
        elif command == "validate-runtime-kinematics":
            for name in ("urdf", "stage", "config", "acceptance", "errors_csv"):
                option(name, required=True)
        elif command == "run-nvidia-sysid":
            option("config", required=True)
            option("execute", action="store_true")
            option("log", default="outputs/nvidia_sysid/official_tool.log")
        elif command == "validate-dynamics":
            for name in ("real_data", "real_manifest", "sim_data", "sim_manifest", "acceptance"):
                option(name, required=True)
            option("errors_csv")
        elif command == "generate-report":
            option("results", default="outputs")
            option("destination", default="deliverables/D05_reports")
        elif command == "package-delivery":
            option("config", required=True)
            option("destination", required=True)
            option("final", action="store_true")
    return root


def main(argv=None):
    args = parser().parse_args(argv)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    destination = Path(args.output or f"outputs/{stamp}_{args.command}.json")
    sdk_required = args.command in ("build-isaac-asset", "validate-runtime-kinematics") or (args.command == "run-test" and args.target == "sim") or (
        args.command == "validate-isaac-asset" and importlib.util.find_spec("pxr") is None)
    if sdk_required and not args.sdk_worker:
        # A plain parent translates Kit's process exit into this toolkit's
        # result status, including failure when Kit exits before producing JSON.
        invocation = list(argv if argv is not None else sys.argv[1:])
        invocation += ["--sdk-worker", "--output", str(destination.resolve())]
        previous = destination.read_bytes() if destination.is_file() else None
        completed = subprocess.run([sys.executable, "-m", "core.cli", *invocation], check=False)
        if not destination.is_file() or (previous is not None and destination.read_bytes() == previous):
            report = result(args.command, {"error": f"SDK worker produced no fresh result (exit {completed.returncode}); inspect its startup log"}, status="blocked")
            report["methodology"] = basis(args.command)
            save(destination, report)
        else:
            report = load(destination)
        return exit_code(report["status"])
    try:
        module = import_module("programs." + COMMANDS[args.command][0])
        report = module.run(args)
    except Exception as exc:
        if args.debug:
            raise
        report = result(args.command, {"error": str(exc), "error_type": type(exc).__name__}, status="blocked")
    report["methodology"] = basis(args.command)
    report["run_id"] = stamp
    save(destination, report)
    print(f"{report['command']}: {report['status']}\nResult: {destination}")
    if report.get("error"):
        print(report["error"], file=sys.stderr)
    code = exit_code(report["status"])
    if args.sdk_worker:
        sys.stdout.flush()
        sys.stderr.flush()
        from adapters.isaac_adapter import shutdown_applications
        shutdown_applications()
    return code


def exit_code(status):
    return 2 if status in ("failed", "blocked") else 3 if status in ("pending", "unverified", "draft") else 0


if __name__ == "__main__":
    raise SystemExit(main())
