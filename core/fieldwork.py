"""Project orchestration; no robot motion and no fabricated measurement values."""
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace
import importlib.metadata
import json
import os
import platform
import re
import shutil
import subprocess
import sys
from zipfile import ZipFile, ZIP_DEFLATED

from core.io import InputError, load, save, sha256, portable_inputs, evidence_paths
from core.model import Model
from core.schemas import parameters

DELIVERIES = {
    "D01": "已确认参数及来源", "D02": "URDF及坐标修正", "D03": "Isaac资产与配置",
    "D04": "程序、接口和原始数据", "D05": "独立验证报告", "D06": "操作与复现说明",
}


@contextmanager
def at(path):
    previous = Path.cwd()
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(previous)


def relative_file(project, value):
    path = Path(value)
    if path.is_absolute() or ".." in path.parts:
        raise InputError("项目文件使用相对路径，不能包含 ..")
    target = (project / path).resolve()
    if not target.is_relative_to(project.resolve()):
        raise InputError("项目文件不能指向项目外部")
    return target


def revision(value):
    if not re.fullmatch(r"[A-Za-z0-9_-]{1,40}", value):
        raise InputError("版本名仅使用字母、数字、下划线或横线")
    return value


def project_config(project):
    return load(project / "project.yaml")


def jobs():
    """Every argument is project-relative; {revision} is replaced at invocation."""
    result = {}

    def add(name, command, **options):
        result[name] = {"command": command, "options": options}

    r, d, c = "results/{revision}/", "data/", "configs/"
    nominal = "models/nominal_{revision}/robot.urdf"
    add("structure", "validate-structure", parameters="parameters/robot_parameters.xlsx", mechanisms=c+"mechanisms.yaml")
    add("nominal", "build-urdf", parameters="parameters/robot_parameters.xlsx", baseline="{baseline}", mechanisms=c+"mechanisms.yaml", destination="models/nominal_{revision}")
    add("reference-fit", "calibrate-joint-reference", data=d+"joint_reference_train.csv", manifest=d+"joint_reference_train.yaml", f_scale=None)
    add("reference-check", "validate-joint-reference", data=d+"joint_reference_validation.csv", manifest=d+"joint_reference_validation.yaml", calibration=r+"reference-fit.json", acceptance=c+"joint_reference_acceptance.yaml", urdf=nominal, mapping_output="models/nominal_{revision}/feedback_mapping.yaml")
    add("feedback", "map-joint-feedback", data=d+"joint_states.csv", manifest=d+"joint_states.yaml", mapping="models/nominal_{revision}/feedback_mapping.yaml", urdf=nominal, output_data=d+"joint_states_mapped_{revision}.csv", output_manifest=d+"joint_states_mapped_{revision}.yaml")
    add("limits", "verify-joint-limits", data=d+"limits.csv", manifest=d+"limits.yaml", specification=c+"limits.yaml")
    add("fit", "calibrate-kinematics", data=d+"kinematics_train.csv", manifest=d+"kinematics_train.yaml", urdf=nominal, config=c+"calibration.yaml")
    add("holdout", "validate-kinematics", data=d+"kinematics_validation.csv", manifest=d+"kinematics_validation.yaml", urdf=nominal, calibration=r+"fit.json", acceptance=c+"acceptance.yaml", errors_csv=r+"holdout_errors.csv")
    corrected = "models/calibrated_{revision}/robot.urdf"
    add("inertia", "validate-structure", parameters="parameters/robot_parameters.xlsx", mechanisms=c+"mechanisms.yaml", require_inertia=True)
    add("isaac", "build-isaac-asset", urdf=corrected, config="models/calibrated_{revision}/isaac_applied.yaml", destination="simulation/{revision}")
    add("asset", "validate-isaac-asset", urdf=corrected, stage="simulation/{revision}/scene.usda", acceptance=c+"acceptance.yaml")
    add("runtime", "validate-runtime-kinematics", urdf=corrected, stage="simulation/{revision}/scene.usda", config=c+"runtime_kinematics.yaml", acceptance=c+"acceptance.yaml", errors_csv=r+"runtime_errors.csv")
    add("sysid", "run-nvidia-sysid", config=c+"nvidia_sysid.yaml", log=r+"sysid.log")
    add("dynamics-fit", "identify-dynamics", data=d+"dynamics_train.csv", manifest=d+"dynamics_train.yaml", urdf=corrected, config=c+"dynamics.yaml")
    add("drives-fit", "tune-drives", data=d+"drives_train.csv", manifest=d+"drives_train.yaml")
    add("contact-fit", "identify-contact", data=d+"contact_train.csv", manifest=d+"contact_train.yaml", config=c+"contact.yaml")
    add("response", "validate-dynamics", real_data=d+"response_real.csv", real_manifest=d+"response_real.yaml", sim_data=d+"response_sim.csv", sim_manifest=d+"response_sim.yaml", acceptance=c+"acceptance.yaml", errors_csv=r+"response_errors.csv")
    add("report", "generate-report", results="results/{revision}", destination="reports/{revision}")
    return result


def write_header(path, fields):
    import csv
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        csv.writer(f).writerow(fields)


def init_project(a, root):
    project = Path(a.project).resolve()
    archive = Path(a.archive).resolve()
    if project.exists() and any(project.iterdir()):
        raise InputError("项目目录必须不存在或为空，避免覆盖资料")
    if not archive.is_file():
        raise InputError("找不到原始模型 ZIP")
    project.mkdir(parents=True, exist_ok=True)
    for name in ("originals", "parameters", "data/raw", "results", "models", "reports", "delivery"):
        (project / name).mkdir(parents=True, exist_ok=True)
    shutil.copytree(root / "configs", project / "configs")
    originals = []
    for source in (archive, Path(a.docx).resolve() if a.docx else None, Path(a.step).resolve() if a.step else None):
        if source:
            if not source.is_file():
                raise InputError(f"找不到原始资料：{source}")
            dest = project / "originals" / source.name
            shutil.copy2(source, dest)
            originals.append({"path": dest.relative_to(project).as_posix(), "sha256": sha256(dest)})
    from programs.common.prepare_inputs import run
    prepared = run(SimpleNamespace(archive=str(project/"originals"/archive.name), destination=str(project/"baseline"), workbook=str(project/"parameters/robot_parameters.xlsx")))
    baseline = Path(prepared["baseline_urdf"])
    model = Model(baseline)
    for key in ("baseline_urdf", "workbook"):
        prepared[key] = Path(prepared[key]).relative_to(project).as_posix()
    save(project / "results/preparation.json", portable_inputs(prepared, project / "results/preparation.json"))
    save(project / "project.yaml", {"schema_version": 1, "project_name": project.name, "robot_id": None,
        "baseline": baseline.relative_to(project).as_posix(), "originals": originals,
        "units": "m rad kg s", "status": "unconfirmed", "scope": list(DELIVERIES),
        "note": "待确认不填0；OpenArm仅软件验证；所有项目路径相对本目录。"})
    save(project / "configs/jobs.yaml", {"jobs": jobs()})
    save(project / "configs/runtime_kinematics.yaml", {"expected_isaac_sim_version": "6.1.0", "physics_backend": "physx",
        "headless": True, "joint_positions": [], "note": "按 docs/mujoco_validation.md 与 runtime 程序定义填写代表姿态，不能越过确认范围。"})
    fields = {
        "joint_reference_train": ["sample_id", "joint", "raw", "reference"],
        "joint_reference_validation": ["sample_id", "joint", "raw", "reference"],
        "limits": ["sample_id", "joint", "boundary", "angle", "direction", "repeat", "event_time", "stop_position"],
        "kinematics_train": ["sample_id", *["q:"+n for n in model.independent], "x", "y", "z", "qx", "qy", "qz", "qw"],
        "kinematics_validation": ["sample_id", *["q:"+n for n in model.independent], "x", "y", "z", "qx", "qy", "qz", "qw"],
        "joint_states": ["sequence", "sample_time", "joint", "q", "dq", "effort"],
        "poses": ["sample_id", "sample_time", "x", "y", "z", "qx", "qy", "qz", "qw", "quality"],
        "dynamics_train": ["sample_id", "time", *[f"{k}:{n}" for n in model.independent for k in ("q", "dq", "ddq", "tau")]],
        "drives_train": ["sample_id", "joint", "q", "dq", "q_command", "dq_command", "pd_effort", "saturated"],
        "contact_train": ["sample_id", "normal_force", "slip_velocity", "tangential_effort", "penetration", "closing_velocity", "approach_speed", "separation_speed"],
        "response_real": ["t", "joint", "q", "dq", "tau", "normal_force", "slip_velocity"],
        "response_sim": ["t", "joint", "q", "dq", "tau", "normal_force", "slip_velocity"],
    }
    base = load(root / "templates/dataset_manifest.yaml")
    for name, columns in fields.items():
        write_header(project / "data" / (name+".csv"), columns)
        meta = dict(base, dataset_id=None, role=None, data_kind=None, robot_id=None,
                    observed_frame=None, pose_source=None, reference_instrument=None, procedure_reference=None)
        if name.startswith("joint_reference"):
            meta.update(reference_kind=None, reference_definition=None, raw_unit=None, reference_unit=None,
                        feedback_coordinates_id=None, urdf_sha256=None)
        if name == "limits":
            meta["event_kind"] = None
        if name in ("response_real", "response_sim"):
            meta.update(test_case_id=None, input_sha256=None, control_mode=None, payload_id=None, initial_state=None, time_basis=None)
        save(project / "data" / (name+".yaml"), meta)
    save(project / "configs/pairing.yaml", {"feedback": "data/joint_states.csv", "poses": "data/poses.csv",
        "feedback_manifest": "data/joint_states.yaml", "pose_manifest": "data/poses.yaml",
        "output_data": "data/kinematics_train.csv", "output_manifest": "data/kinematics_train.yaml",
        "role": None, "dataset_id": None, "observed_frame": None,
        "maximum_time_difference_s": None, "pose_time_offset_s": None, "time_sync_reference": None,
        "pose_length_unit": None, "joint_angle_unit": None, "joint_length_unit": None,
        "required_quality": "valid", "note": "只配对已换到同一observed_frame的位姿；不能用相同URDF生成测量真值；未实现自动设备格式识别。"})
    save(project/"configs/pose_transform.yaml", {"marker_poses": "data/raw/tool_marker.csv", "marker_manifest": "data/raw/tool_marker.yaml",
        "base_marker_poses": "data/raw/base_marker.csv", "base_marker_manifest": "data/raw/base_marker.yaml",
        "base_to_base_marker_xyz_rpy": None, "tool_to_tool_marker_xyz_rpy": None, "installation_reference": None,
        "input_length_unit": None, "maximum_time_difference_s": None, "required_quality": "valid", "observed_frame": None,
        "output_data": "data/poses.csv", "output_manifest": "data/poses.yaml"})
    save(project/"configs/offset_feedback.yaml", {"data": "data/joint_states_mapped_R01.csv", "manifest": "data/joint_states_mapped_R01.yaml",
        "fit_input_coordinates_id": None, "output_data": "data/joint_states_calibrated_R01.csv", "output_manifest": "data/joint_states_calibrated_R01.yaml"})
    make_record_book(project / "parameters/现场测量与交付记录.xlsx")
    print(f"项目已建立：{project}\n先阅读 docs/现场操作手册.html，再填写 parameters 与 configs。")
    return 0


def make_record_book(path):
    from openpyxl import Workbook
    from openpyxl.styles import Font, PatternFill, Alignment
    book = Workbook()
    book.remove(book.active)
    tables = {
        "资料接收": [["项目", "版本/型号", "文件/页码", "是否匹配STEP", "负责人", "状态", "备注"],
                 ["结构主表 STEP候选连杆与连接关系确认.xlsx", None, None, None, None, "待提供", "此记录表不冒充原始结构主表"]],
        "结构对应": [["候选组C/A/B/W", "零件P编号", "原零件名", "装配路径", "候选连接K编号", "实际link", "实际joint", "父link", "子link", "固定/可动/闭环", "正确/需修改/未知", "来源", "备注"]],
        "轴线零位": [["K编号", "joint", "参考坐标", "轴上一点x_mm", "y_mm", "z_mm", "方向ax", "ay", "az", "正向图/文件", "STEP姿态是否零位", "零位差deg或mm", "依据及版本", "状态"]],
        "范围额定": [["joint", "类型", "正常lower_deg或mm", "正常upper_deg或mm", "软件边界", "开关触发", "机械边界记录", "连续旋转", "速度及单位", "加速度及单位", "力矩额定/峰值/运行及单位", "负载与控制模式", "来源/页码", "状态"]],
        "测量设备": [["设备ID", "厂家型号", "序列号", "量程", "精度/不确定度及条件", "分辨率", "校准记录", "有效期", "安装工装", "采样/时钟/触发", "SDK/版本", "文件"]],
        "坐标安装": [["父坐标", "子坐标", "T_parent_child", "xyz_m", "rpy_rad或xyzw", "固定方式", "测量依据", "设备/日期", "状态"]],
        "试验与负载": [["case_id", "机器人型号版本", "步骤规程", "初始状态", "控制模式", "实际命令文件及哈希", "工具/负载", "质量/质心/惯性来源", "表面/材料", "同步依据", "标定或验证", "数据文件"]],
        "验收与交付": [["编号", "内容", "已完成/待完成/失败/不适用", "证据文件", "指标/条件", "负责人/日期", "缺项或不适用原因"], *[[k,v,"待完成",None,None,None,None] for k,v in DELIVERIES.items()]],
    }
    catalog=load(Path(__file__).resolve().parents[1]/"docs/field_catalog.json")
    tables["参数覆盖与签署"]=[["编号","参数类别","完整范围","工具/资料","应用位置","对应交付","来源/页码","测量文件","独立验证报告","状态","确认人/日期"],
        *[[r[0],r[1],r[2],r[3],r[6],r[7],None,None,None,"待完成",None] for r in catalog["sheets"]["参数工具矩阵"][1:]]]
    tables["工具准备状态"]=[["编号","工具能力","配置","参数覆盖","已备/待备/不适用","具体型号或资料","校准/能力文件","负责人"],
        *[[r[0],r[1],r[2],r[7],"待确认",None,None,None] for r in catalog["sheets"]["工具准备清单"][1:]]]
    for name, rows in tables.items():
        sheet = book.create_sheet(name)
        for row in rows:
            sheet.append(row)
        sheet.freeze_panes = "A2"
        sheet.auto_filter.ref = sheet.dimensions
        for c in sheet[1]:
            c.font = Font(color="FFFFFF", bold=True)
            c.fill = PatternFill("solid", fgColor="215A8E")
        for col in sheet.columns:
            sheet.column_dimensions[col[0].column_letter].width = 24
            for cell in col:
                cell.alignment = Alignment(wrap_text=True, vertical="top")
    book.save(path)


def doctor(project=None):
    versions = {}
    for name in ("numpy", "scipy", "PyYAML", "openpyxl", "mujoco", "trimesh", "usd-core", "pytest"):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    out = {"platform": platform.platform(), "architecture": platform.machine(), "libc": platform.libc_ver(),
        "python": sys.version, "executable": sys.executable, "packages": versions,
        "core_ready": all(versions[n] for n in ("numpy", "scipy", "PyYAML", "openpyxl")),
        "isaac_bundled": False, "hardware_adapter_validated": False, "robot_motion_sent": False}
    if project:
        machine = project / ".machine.json"
        cfg = load(machine) if machine.exists() else {}
        sdk = cfg.get("isaac_python")
        out["isaac_python_configured"] = bool(sdk and Path(sdk).is_file())
        out["isaac_note"] = "Isaac 6.1/GPU/驱动/扩展另行安装并实测；配置路径只属于本机，不参与项目数据定义。"
    return out


def run_job(project, name, rev, root):
    cfg = project_config(project)
    job = load(project / "configs/jobs.yaml")["jobs"].get(name)
    if not job:
        raise InputError("未知 job，请运行 status 查看")
    from core.cli import COMMANDS
    command = job.get("command")
    if command not in COMMANDS or command in ("check-openarm", "run-test", "capture-data"):
        raise InputError("现场 job 不执行设备操作；设备采集/动作通过已验证适配器与明确命令单独运行")
    dest = project / f"results/{rev}/{name}.json"
    if dest.exists():
        raise InputError("此版本的结果已存在；修改参数后使用 --revision R02 等新版本，保留原证据")
    args = [command]
    for key, value in job.get("options", {}).items():
        if value is None:
            raise InputError(f"请填写 configs/jobs.yaml → jobs.{name}.options.{key}")
        flag = "--" + key.replace("_", "-")
        if isinstance(value, bool):
            if value:
                args.append(flag)
        else:
            values = value if isinstance(value, list) else [value]
            expanded = [str(v).format(revision=rev, baseline=cfg["baseline"]) for v in values]
            if "--execute" in expanded or key == "execute":
                raise InputError("现场 job 不开放 execute；参照适用项目/部件规程使用底层命令")
            args.extend([flag, *expanded])
    args.extend(["--output", str(dest)])
    sdk_jobs = {"build-isaac-asset", "validate-runtime-kinematics", "run-nvidia-sysid"}
    python = sys.executable
    env = dict(os.environ, PYTHONPATH=str(root), PYTHONNOUSERSITE="1")
    if command in sdk_jobs:
        machine_path = project / ".machine.json"
        machine = load(machine_path) if machine_path.exists() else {}
        python = machine.get("isaac_python")
        if not python or not Path(python).is_file():
            raise InputError('Isaac未配置：在项目.machine.json填写 {"isaac_python":"本机6.1的python.sh或python路径"}')
        # Core offline packages must not replace the SDK's tested scientific stack.
        for key in ("CONDA_PREFIX", "ISAAC_PATH", "EXP_PATH", "CARB_APP_PATH", "ROBOTCAL_SITE"):
            env.pop(key, None)
    dest.parent.mkdir(parents=True, exist_ok=True)
    with (dest.with_suffix(".log")).open("w", encoding="utf-8") as log:
        done = subprocess.run([python, str(root/"run.py"), *args], cwd=project, env=env, stdout=log, stderr=subprocess.STDOUT)
    print(dest.with_suffix(".log").read_text(encoding="utf-8")[-6000:])
    return done.returncode


def seal(project, data, manifest):
    csv = relative_file(project, data)
    meta_path = relative_file(project, manifest)
    from core.io import csv_rows
    meta = load(meta_path)
    for key in ("dataset_id", "role", "data_kind", "clock", "source"):
        if not meta.get(key):
            raise InputError(f"先填写 {manifest} → {key}，不能自动把未知数据标成真机数据")
    csv_rows(csv)
    if meta.get("sha256"):
        raise InputError("数据已有封存哈希；原文件保留，另存新版本后重新封存")
    meta["sha256"] = sha256(csv)
    save(meta_path, meta)
    print("已封存：", csv, meta["sha256"])
    return 0


def edit_parameter(a, project):
    from openpyxl import load_workbook
    path = project / "parameters/robot_parameters.xlsx"
    book = load_workbook(path)
    sheet = book[a.sheet]
    headers = [c.value for c in sheet[1]]
    identity = "joint" if a.sheet == "actuators" else "name"
    if a.field not in headers or a.field in (identity, "status", "source", "inertial_status", "inertial_source"):
        raise InputError("字段不存在或需通过表格单独确认；本命令不自动更改确认状态")
    index = headers.index(a.field) + 1
    rows = [r for r in range(2, sheet.max_row+1) if sheet.cell(r, headers.index(identity)+1).value == a.name]
    if len(rows) != 1:
        raise InputError("对象名必须唯一且已存在")
    value = json.loads(a.value)
    if not isinstance(value, (str, int, float, list)) or isinstance(value, bool):
        raise InputError("请输入JSON数字、字符串或数组；未知值应保留空白")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    backup = project / f"parameters/history/{stamp}_robot_parameters.xlsx"
    backup.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(path, backup)
    old = sheet.cell(rows[0], index).value
    sheet.cell(rows[0], index).value = " ".join(map(str, value)) if isinstance(value, list) else value
    if "source" in headers:
        sheet.cell(rows[0], headers.index("source")+1).value = a.source
    if a.sheet == "links" and a.field in ("mass", "com_xyz", "com_rpy", "inertia"):
        sheet.cell(rows[0], headers.index("inertial_status")+1).value = "unconfirmed"
        sheet.cell(rows[0], headers.index("inertial_source")+1).value = a.source
    elif "status" in headers:
        sheet.cell(rows[0], headers.index("status")+1).value = "unconfirmed"
    book.save(path)
    save(project / f"parameters/history/{stamp}_change.json", {"sheet": a.sheet, "name": a.name, "field": a.field,
        "old": old, "new": value, "source": a.source, "backup": backup.relative_to(project).as_posix(), "status": "requires_review"})
    print("已修改并备份；请审核后在表中确认，再用新 revision 重建与验证。")
    return 0


def status(project, rev):
    cfg = project_config(project)
    from core.schemas import validate_parameters
    checked = validate_parameters(parameters(project / "parameters/robot_parameters.xlsx"))
    summary = {"project": project.name, "revision": rev, "baseline": cfg["baseline"],
        "structure_errors": checked["errors"], "pending_parameters": checked["pending"],
        "jobs": list(load(project / "configs/jobs.yaml")["jobs"]),
        "results": {p.stem: load(p).get("status") for p in (project / f"results/{rev}").glob("*.json")},
        "all_deliveries": DELIVERIES, "accepted_robot": False}
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    save(project / f"reports/{rev}/readiness.json", summary)
    return 0


def backup(project, destination):
    destination = Path(destination).resolve()
    if destination.exists() or destination.is_relative_to(project):
        raise InputError("备份写到项目外部的新ZIP文件")
    files = [p for p in project.rglob("*") if p.is_file() and not p.is_symlink() and p.name != ".machine.json"]
    destination.parent.mkdir(parents=True, exist_ok=True)
    with ZipFile(destination, "w", ZIP_DEFLATED) as z:
        for p in files:
            z.write(p, project.name + "/" + p.relative_to(project).as_posix())
        z.writestr(project.name+"/BACKUP_MANIFEST.json", json.dumps({"files": [{"path": p.relative_to(project).as_posix(), "sha256": sha256(p)} for p in files]}, ensure_ascii=False, indent=2))
    print("备份完成：", destination)
    return 0


def export(project, rev, root, destination, final):
    from programs.common.package_delivery import run
    if final:
        from core.field_validation import checked_fit
        checked_fit(project, rev)
        applied_path = project/f"results/{rev}/verify_applied.json"
        applied = load(applied_path)
        evidence_paths(applied, applied_path)
        if applied.get("command") != "field-verify-applied" or applied.get("status") != "passed" or not applied.get("is_real_robot_acceptance"):
            raise InputError("正式现场交付需要实际交付模型的真实独立verify-applied通过记录")
    readiness = project / f"reports/{rev}/D03_readiness.json"
    save(readiness, {"status": "requires_project_review", "isaac_available": (project/f"simulation/{rev}/scene.usda").exists(),
        "note": "正式交付前核对闭环、负载、接触、同步、独立验证及完整性。此文件不是D03通过证明。"})
    results = [str(p.relative_to(project)) for p in sorted((project / f"results/{rev}").glob("*.json"))]
    artifacts = [
        {"delivery": "D01", "path": "parameters"}, {"delivery": "D01", "path": "configs"},
        {"delivery": "D01", "path": "originals"}, {"delivery": "D02", "path": "models"},
        {"delivery": "D03", "path": f"simulation/{rev}" if (project/f"simulation/{rev}").exists() else str(readiness.relative_to(project))},
        {"delivery": "D04", "path": "data"}, {"delivery": "D04", "path": str(root/"core")},
        {"delivery": "D04", "path": str(root/"programs")}, {"delivery": "D04", "path": str(root/"adapters")},
        {"delivery": "D04", "path": str(root/"examples")}, {"delivery": "D04", "path": str(root/"field.py")},
        {"delivery": "D04", "path": str(root/"run.py")}, {"delivery": "D04", "path": str(root/"pyproject.toml")},
        {"delivery": "D04", "path": str(root/"启动工具包.sh")},
        {"delivery": "D04", "path": str(root/"motion.py")},
        {"delivery": "D04", "path": str(root/"控制关节.sh")},
        {"delivery": "D05", "path": f"results/{rev}"}, {"delivery": "D05", "path": f"reports/{rev}"},
        {"delivery": "D06", "path": str(root/"docs")},
    ]
    config = {"stage": load(project/"configs/delivery.yaml").get("stage", "kinematics"),
        "required_dynamics_metrics": load(project/"configs/delivery.yaml").get("required_dynamics_metrics", []),
        "reports": results, "artifacts": artifacts}
    config_path = project / f"delivery/{rev}.yaml"
    save(config_path, config)
    target=Path(destination).resolve()
    with at(project):
        report = run(SimpleNamespace(config=str(config_path), destination=str(target), final=final))
    save(project / f"delivery/{rev}_package.json", portable_inputs(report, project/f"delivery/{rev}_package.json"))
    print("分类归档完成；可复运行的完整项目另用 backup，连同离线工具包交接。")
    return 0


def dispatch(a, root):
    if a.action == "doctor":
        out = doctor(Path(a.project).resolve() if a.project else None)
        print(json.dumps(out, ensure_ascii=False, indent=2))
        if a.output:
            save(a.output, out)
        return 0 if out["core_ready"] else 2
    if a.action == "init":
        return init_project(a, root)
    if a.action == "self-test":
        from core.field_validation import self_test
        return self_test(Path(a.destination).resolve(), root)
    if a.action in ("build-bundle", "check-bundle"):
        from core.portable_bundle import build_bundle, check_bundle
        return build_bundle(root, Path(a.destination).resolve(), Path(a.project).resolve() if a.project else None) if a.action == "build-bundle" else check_bundle(Path(a.root).resolve())
    project = Path(a.project).resolve()
    project_config(project)
    rev = revision(a.revision)
    if a.action == "run":
        return run_job(project, a.job, rev, root)
    if a.action == "status":
        return status(project, rev)
    if a.action == "seal":
        return seal(project, a.data, a.manifest)
    if a.action == "edit":
        return edit_parameter(a, project)
    if a.action == "backup":
        return backup(project, a.destination)
    if a.action == "export":
        return export(project, rev, root, a.destination, a.final)
    if a.action == "preview":
        from examples.run_mujoco_validation import run
        cfg=project_config(project)
        models=[project/f"models/calibrated_{rev}/robot.urdf",project/f"models/nominal_{rev}/robot.urdf",project/cfg["baseline"]]
        urdf=next(p for p in models if p.is_file())
        destination=project/f"simulation/mujoco_{rev}"
        report=run(SimpleNamespace(urdf=urdf,destination=destination,joint_states=None,joint_manifest=None,reference_joints=[]))
        print("仅模型预览和软件检查；测试参数未确认时不能证明真实机构或精度。模型：",urdf)
        if a.viewer:
            return subprocess.run([sys.executable,str(root/"examples/view_mujoco.py"),"--urdf",str(urdf),"--mjcf",str(destination/"model/robot.urdf"),"--poses",str(destination/"view_poses.json")],check=False).returncode
        return 0
    if a.action in ("apply", "verify-applied", "apply-drives"):
        from core.field_validation import apply_kinematics, verify_applied, apply_drives
        return {"apply": apply_kinematics, "verify-applied": verify_applied, "apply-drives": apply_drives}[a.action](project, rev)
    if a.action in ("pair-poses", "transform-poses", "offset-feedback"):
        from core.field_validation import pair_poses, transform_poses, offset_feedback
        if a.action == "offset-feedback":
            return offset_feedback(project, rev, relative_file(project, a.config))
        return {"pair-poses": pair_poses, "transform-poses": transform_poses}[a.action](project, relative_file(project, a.config))
    raise InputError("未知操作")
