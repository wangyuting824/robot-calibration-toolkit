"""Apply measured results once and verify the model that will actually be handed over."""
from pathlib import Path
from types import SimpleNamespace as NS
import json
import math
import shutil
import sys

import numpy as np
from scipy.spatial.transform import Rotation

from core.io import (InputError, load, save, sha256, number, dataset, csv_rows,
                     write_csv, result, portable_inputs, evidence_paths)
from core.model import Model
from core.optimization import predict, decode, POSE_COLUMNS
from core.units_and_frames import pose_from_row, pose_error, transform
from core.metrics import summary


def save_result(path, report):
    save(path, portable_inputs(report, path))
    print(f"{report['command']}: {report['status']}\n{path}")


def fit_paths(project, rev):
    return (project/f"models/nominal_{rev}/robot.urdf", project/f"results/{rev}/fit.json",
            project/f"results/{rev}/holdout.json", project/f"models/calibrated_{rev}")


def checked_fit(project, rev):
    nominal, fit_path, check_path, destination = fit_paths(project, rev)
    fit, check = load(fit_path), load(check_path)
    evidence_paths(fit, fit_path)
    evidence_paths(check, check_path)
    if fit.get("command") != "calibrate-kinematics" or fit.get("diagnostics", {}).get("identifiable") is not True:
        raise InputError("先完成可辨识的运动学拟合")
    if check.get("command") != "validate-kinematics" or check.get("status") not in ("passed", "synthetic_pass", "simulation_pass") or check.get("calibration_sha256") != sha256(fit_path):
        raise InputError("先完成同一拟合版本的独立留出验证；缺少阈值也不能跳过")
    bound = [e for e in fit.get("inputs", []) if e["path"].endswith(".urdf")]
    if len(bound) != 1 or bound[0]["sha256"] != sha256(nominal):
        raise InputError("拟合模型与本版本名义模型不一致")
    manifest=load(nominal.parent/"model_manifest.yaml")
    if manifest.get("urdf_sha256") != sha256(nominal) or manifest.get("parameters_sha256") != sha256(project/"parameters/robot_parameters.xlsx"):
        raise InputError("参数主表或名义模型已改变，需新版本重新构建与验证")
    return nominal, fit_path, check_path, destination, fit


def apply_kinematics(project, rev):
    nominal, fit_path, check_path, dest, fit = checked_fit(project, rev)
    if dest.exists():
        raise InputError("修正模型目录已存在；更改后另建revision，不能覆盖证据")
    from core.fieldwork import project_config
    from programs.stage_01_definition.build_urdf import run
    cfg = project_config(project)
    args = dict(parameters=str(project/"parameters/robot_parameters.xlsx"), baseline=str(project/cfg["baseline"]),
                mechanisms=str(project/"configs/mechanisms.yaml"), calibration=None, require_inertia=False, draft=False)
    # Rebuild the nominal inputs, so a changed workbook cannot silently consume an old fit.
    temp = project/f"models/.nominal_check_{rev}"
    if temp.exists():
        raise InputError("临时检查目录已存在，请先保留并检查上次失败原因")
    try:
        run(NS(**args, destination=str(temp)))
        if sha256(temp/"robot.urdf") != sha256(nominal):
            raise InputError("参数表已变化；请新建revision重新生成名义模型、拟合与留出验证")
    finally:
        if temp.exists():
            shutil.rmtree(temp)
    args["calibration"] = str(fit_path)
    report = run(NS(**args, destination=str(dest)))
    base, tool, offsets, _ = decode(fit["config"], fit["specifications"], fit["values"])
    frame = {"convention": "T_parent_child", "units": "m rad", "observed_frame": fit["config"]["observed_frame"],
        "root_link": Model(nominal).root, "tip_link": fit["config"]["tip_link"],
        "base_xyz_rpy": base.tolist(), "tool_xyz_rpy": tool.tolist(),
        "joint_offset_convention": "q_model = q_input_to_kinematic_fit + joint_offset",
        "joint_offsets": offsets, "note": "origin修正已写URDF；这些offset只用于数据/控制边界一次；工具负载惯性仍需单独建模。"}
    save(dest/"scene_frames.yaml", frame)
    isaac = load(project/"configs/isaac.yaml")
    # A revision-specific config avoids modifying the reviewed source configuration.
    isaac["base_xyz_rpy"] = base.tolist()
    isaac["tool_frame"] = {"parent_link": frame["tip_link"], "xyz_rpy": tool.tolist(), "name": "calibrated_tool"}
    save(dest/"isaac_applied.yaml", isaac)
    save(dest/"APPLICATION.json", {"schema_version": 1, "nominal_urdf_sha256": sha256(nominal),
        "calibration_sha256": sha256(fit_path), "holdout_sha256": sha256(check_path),
        "corrected_urdf_sha256": sha256(dest/"robot.urdf"), "scene_frames_sha256": sha256(dest/"scene_frames.yaml"),
        "joint_offsets_sha256": sha256(dest/"joint_offsets.yaml"),
        "joint_offsets_applied_to_origins": False, "real_robot_updated": False,
        "feedback_mapping_rebound_automatically": False,
        "note": "反馈映射仍绑定名义模型。不能编辑哈希冒充重新验证；本次附加零偏相对于拟合CSV的关节坐标。"})
    save_result(project/f"results/{rev}/apply.json", result("field-apply", {"model": str(dest.relative_to(project)),
        "status_detail": "必须继续 verify-applied；Isaac job配置改为修正目录中的isaac_applied.yaml",
        "is_real_robot_acceptance": False}, [nominal, fit_path, check_path, dest/"robot.urdf", dest/"scene_frames.yaml"], "built"))
    return 0


def verify_applied(project, rev):
    nominal, fit_path, check_path, dest, fit = checked_fit(project, rev)
    app = load(dest/"APPLICATION.json")
    for key, path in (("corrected_urdf_sha256", dest/"robot.urdf"), ("scene_frames_sha256", dest/"scene_frames.yaml"),
                      ("joint_offsets_sha256", dest/"joint_offsets.yaml"), ("calibration_sha256", fit_path), ("holdout_sha256", check_path)):
        if sha256(path) != app[key]:
            raise InputError(f"应用后的文件已变化：{path}，需新版本重新应用与验证")
    original, corrected = Model(nominal), Model(dest/"robot.urdf")
    paths = evidence_paths(load(check_path), check_path)
    csv = [p for p in paths if p.suffix == ".csv"]
    if len(csv) != 1:
        raise InputError("不能唯一识别留出验证CSV")
    rows = csv_rows(csv[0], ["sample_id", *POSE_COLUMNS, *["q:"+n for n in original.independent]])
    frame = load(dest/"scene_frames.yaml")
    offsets = load(dest/"joint_offsets.yaml")["offsets"]
    applied_config = {"tip_link": frame["tip_link"], "base_xyz_rpy": frame["base_xyz_rpy"], "tool_xyz_rpy": frame["tool_xyz_rpy"]}
    specs = [{"kind": "joint_offset", "joint": n} for n in offsets]
    values = list(offsets.values())
    errors, baseline_errors, equivalent = [], [], []
    for row in rows:
        intended = predict(original, row, fit["config"], fit["specifications"], fit["values"])
        actual = predict(corrected, row, applied_config, specs, values)
        observed = pose_from_row(row)
        equivalent.append(pose_error(actual, intended))
        errors.append(pose_error(actual, observed))
        baseline_errors.append(pose_error(predict(original, row, fit["config"]), observed))
    errors, equivalent, baseline_errors = map(np.array, (errors, equivalent, baseline_errors))
    p, a = np.linalg.norm(errors[:, :3], axis=1), np.linalg.norm(errors[:, 3:], axis=1)
    ep, ea = np.linalg.norm(equivalent[:, :3], axis=1), np.linalg.norm(equivalent[:, 3:], axis=1)
    # These tolerances only check numerical application, never physical accuracy.
    application_ok = max(ep) < 1e-9 and max(ea) < 1e-9
    acceptance_path = project/"configs/acceptance.yaml"
    thresholds = load(acceptance_path)["kinematics"]
    limits = (number(thresholds.get("max_position_m")), number(thresholds.get("max_orientation_rad")))
    if min(limits) <= 0:
        raise InputError("正式物理误差阈值须为正数")
    accepted = application_ok and max(p) <= limits[0] and max(a) <= limits[1]
    check = load(check_path)
    real = check["dataset"]["data_kind"] == "real" and fit["training_dataset"]["data_kind"] == "real"
    status = "failed" if not accepted else "passed" if real else "synthetic_pass" if check["dataset"]["data_kind"] == "synthetic" else "simulation_pass"
    metrics = {"position_m": summary(p), "orientation_rad": summary(a)}
    baseline = {"position_m": summary(np.linalg.norm(baseline_errors[:, :3], axis=1)),
                "orientation_rad": summary(np.linalg.norm(baseline_errors[:, 3:], axis=1))}
    path = project/f"results/{rev}/verify_applied.json"
    write_csv(path.with_suffix(".csv"), [{"sample_id": row["sample_id"], "position_error_m": float(pi),
        "orientation_error_rad": float(ai), "application_difference_m": float(ei)} for row,pi,ai,ei in zip(rows,p,a,ep)])
    save_result(path, result("field-verify-applied", {"metrics": metrics, "before_metrics": baseline,
        "application_equivalence": {"max_position_m": float(max(ep)), "max_angle_rad": float(max(ea)), "passed": application_ok},
        "dataset": check["dataset"], "training_data_kind": fit["training_dataset"]["data_kind"],
        "is_real_robot_acceptance": bool(real and accepted), "hardware_commands_sent": False,
        "scope": "交付URDF加独立offset和坐标文件复算；并非实机控制器更新或仿真动力学验收"},
        [nominal, fit_path, check_path, csv[0], dest/"robot.urdf", dest/"scene_frames.yaml", dest/"joint_offsets.yaml", acceptance_path], status))
    return 0 if accepted else 2


def apply_drives(project, rev):
    path = project/f"results/{rev}/drives-fit.json"
    report = load(path)
    evidence_paths(report, path)
    if report.get("command") != "tune-drives" or report.get("status") != "computed":
        raise InputError("需要成功的 isolated_pd_effort 回归结果")
    model = Model(project/f"models/calibrated_{rev}/robot.urdf")
    base_config = project/f"models/calibrated_{rev}/isaac_applied.yaml"
    cfg = load(base_config)
    configured = cfg.get("drives", {})
    for name, value in report["joints"].items():
        if name not in model.independent or not value.get("diagnostics", {}).get("identifiable"):
            raise InputError("驱动对象或辨识状态不正确")
        if name not in configured or configured[name].get("max_effort") is None:
            raise InputError(f"先在 {base_config} 的 drives.{name}.max_effort 填厂家允许运行限值")
        if number(configured[name]["max_effort"]) <= 0:
            raise InputError("max_effort必须为正数")
        configured[name].update(type="force", stiffness=value["stiffness"], damping=value["damping"])
    cfg["drives"] = configured
    cfg["candidate_gain_source_sha256"] = sha256(path)
    destination = base_config.with_name("isaac_drives_candidate.yaml")
    if destination.exists():
        raise InputError("候选驱动配置已存在；另建revision")
    save(destination, cfg)
    print(f"候选驱动配置：{destination}。改Isaac job配置后重新导入，用匹配输入的独立动态试验验收。")
    return 0


def pair_poses(project, config_path):
    """Time-nearest full feedback frames; no interpolation, offset fitting, or silent drops."""
    from core.fieldwork import relative_file, project_config
    cfg = load(config_path)
    required = ("dataset_id", "role", "observed_frame", "time_sync_reference", "required_quality")
    if any(not cfg.get(k) for k in required) or cfg["role"] not in ("calibration", "validation"):
        raise InputError("配对需要数据ID、role、坐标、质量标志和已有同步依据")
    tolerance = number(cfg.get("maximum_time_difference_s"))
    offset = number(cfg.get("pose_time_offset_s"))
    if tolerance <= 0:
        raise InputError("请根据同步/测量能力声明正的时间容差")
    units = (cfg.get("pose_length_unit"), cfg.get("joint_angle_unit"), cfg.get("joint_length_unit"))
    if units[0] not in ("m", "mm") or units[1] not in ("rad", "deg") or units[2] not in ("m", "mm"):
        raise InputError("显式选择pose长度、关节角、直线关节长度单位")
    length_scale, angle_scale, joint_scale = (0.001 if units[0] == "mm" else 1, math.pi/180 if units[1] == "deg" else 1, 0.001 if units[2] == "mm" else 1)
    feedback_path = relative_file(project, cfg["feedback"])
    poses_path = relative_file(project, cfg["poses"])
    fm = load(relative_file(project, cfg["feedback_manifest"]))
    pm = load(relative_file(project, cfg["pose_manifest"]))
    for meta, path in ((fm, feedback_path), (pm, poses_path)):
        if not meta.get("source") or not meta.get("clock") or not meta.get("sha256") or meta["sha256"] != sha256(path):
            raise InputError("输入来源、时钟及文件哈希必须完整且匹配")
    if fm.get("data_kind") != pm.get("data_kind") or fm.get("data_kind") not in ("real", "synthetic", "simulation"):
        raise InputError("两类输入必须是同一种数据性质，禁止将合成/仿真混入实测")
    if pm.get("observed_frame") != cfg["observed_frame"] or not pm.get("pose_source"):
        raise InputError("位姿需先转换到声明的同一测量坐标，并注明独立位姿来源")
    if not fm.get("coordinates_id") or not pm.get("reference_instrument"):
        raise InputError("需要反馈坐标定义coordinates_id和独立位姿仪器reference_instrument")
    model = Model(project/project_config(project)["baseline"])
    # If topology was updated, pair against the reviewed model instead of the original.
    if cfg.get("urdf"):
        model = Model(relative_file(project, cfg["urdf"]))
    feedback = csv_rows(feedback_path, ["sequence", "sample_time", "joint", "q"])
    poses = csv_rows(poses_path, ["sample_id", "sample_time", *POSE_COLUMNS, "quality"])
    frames = {}
    for row in feedback:
        frame = frames.setdefault(row["sequence"], {})
        if row["joint"] in frame:
            raise InputError("一帧内同名关节重复，不能猜测应使用哪条")
        frame[row["joint"]] = row
    complete = []
    for frame in frames.values():
        if set(model.independent) - set(frame):
            raise InputError("关节反馈帧缺少独立关节，未自动补0")
        times = [number(frame[n]["sample_time"]) for n in model.independent]
        if max(times) - min(times) > tolerance:
            raise InputError("同一反馈帧中关节测量时间跨度超过容差")
        complete.append((sum(times)/len(times), frame))
    output, audit, used, ids = [], [], set(), set()
    for pose in poses:
        if pose["quality"] != cfg["required_quality"]:
            raise InputError("存在质量不合格位姿；保留原始记录，审核后另存选取文件，不能静默丢弃")
        if not pose["sample_id"] or pose["sample_id"] in ids:
            raise InputError("位姿 sample_id 为空或重复")
        ids.add(pose["sample_id"])
        t = number(pose["sample_time"]) + offset
        index = min(range(len(complete)), key=lambda i: abs(complete[i][0]-t))
        frame_time, frame = complete[index]
        delta = frame_time-t
        if abs(delta) > tolerance or index in used:
            raise InputError("超时或同一反馈帧被重复配对；调整已记录的采集/同步方案后重采")
        used.add(index)
        actual_times = [number(frame[n]["sample_time"]) for n in model.independent]
        if any(abs(ft-t) > tolerance for ft in actual_times):
            raise InputError("某关节的真实采样时间与位姿超过容差")
        pose_from_row(pose)  # Reject invalid/non-normalized quaternion, do not normalize silently.
        row = {"sample_id": pose["sample_id"]}
        for name in model.independent:
            scale = joint_scale if model.joints[name].get("type") == "prismatic" else angle_scale
            row["q:"+name] = number(frame[name]["q"]) * scale
        row.update({k: number(pose[k])*length_scale for k in ("x", "y", "z")})
        row.update({k: number(pose[k]) for k in ("qx", "qy", "qz", "qw")})
        output.append(row)
        audit.append({"sample_id": pose["sample_id"], "feedback_sequence": frame[next(iter(frame))]["sequence"],
            "feedback_time_s": frame_time, "pose_time_in_feedback_clock_s": t, "delta_s": delta})
    dest = relative_file(project, cfg["output_data"])
    meta_path = relative_file(project, cfg["output_manifest"])
    if dest.exists() and dest.stat().st_size and len(dest.read_text(encoding="utf-8").splitlines()) > 1:
        raise InputError("输出数据已存在，使用新文件名；不覆盖采集或先前配对结果")
    write_csv(dest, output)
    manifest = {"dataset_id": cfg["dataset_id"], "role": cfg["role"], "data_kind": pm["data_kind"],
        "units": {"length": "m", "angle": "rad", "time": "s", "effort": "N_or_Nm"},
        "clock": fm["clock"], "source": "离线时间配对；"+pm["source"]+"；"+fm["source"],
        "observed_frame": cfg["observed_frame"], "pose_source": pm["pose_source"],
        "reference_instrument": pm["reference_instrument"], "sha256": sha256(dest),
        "coordinates_id": fm["coordinates_id"], "time_sync_reference": cfg["time_sync_reference"],
        "maximum_time_difference_s": tolerance, "pose_time_offset_s": offset,
        "input_hashes": {"feedback": sha256(feedback_path), "poses": sha256(poses_path)},
        "is_real_robot_acceptance": False}
    save(meta_path, manifest)
    write_csv(dest.with_name(dest.stem+"_pairing_audit.csv"), audit)
    print("配对完成：", dest, "。这里只完成数据整理，仍需模型/测量坐标及独立验收。")
    return 0


def transform_poses(project, config_path):
    """Two measured markers and independently surveyed installation transforms.

    T_B_E = T_B_Bm @ inv(T_M_Bm) @ T_M_Em @ inv(T_E_Em).
    No robot FK is used to manufacture a reference observation.
    """
    from core.fieldwork import relative_file
    from core.io import vector
    cfg = load(config_path)
    if not cfg.get("installation_reference") or not cfg.get("observed_frame"):
        raise InputError("需要工装安装依据和机器人基座测量坐标名")
    bm = vector(cfg.get("base_to_base_marker_xyz_rpy"), 6, "T_base_baseMarker")
    em = vector(cfg.get("tool_to_tool_marker_xyz_rpy"), 6, "T_tool_toolMarker")
    tb, te = transform(bm[:3],bm[3:]), transform(em[:3],em[3:])
    tolerance = number(cfg.get("maximum_time_difference_s"))
    if tolerance <= 0 or cfg.get("input_length_unit") not in ("m", "mm"):
        raise InputError("声明正的同步容差及输入长度单位m/mm；安装变换始终m/rad")
    paths = [relative_file(project, cfg[key]) for key in ("marker_poses", "base_marker_poses", "marker_manifest", "base_marker_manifest")]
    tool_path, base_path, tool_meta, base_meta = paths
    tm, bm_meta = load(tool_meta), load(base_meta)
    for meta, path in ((tm,tool_path),(bm_meta,base_path)):
        if meta.get("sha256") != sha256(path) or not meta.get("pose_source") or not meta.get("reference_instrument"):
            raise InputError("两个靶标都需要已封存的独立测量来源与设备信息")
    for field in ("clock", "observed_frame", "data_kind", "role"):
        if not tm.get(field) or tm[field] != bm_meta.get(field):
            raise InputError("两个靶标必须具有相同仪器坐标、时钟、role和数据性质")
    if tm["data_kind"] not in ("real", "synthetic", "simulation"):
        raise InputError("未知数据性质")
    tool_rows = csv_rows(tool_path,["sample_id","sample_time",*POSE_COLUMNS,"quality"])
    base_rows = csv_rows(base_path,["sample_id","sample_time",*POSE_COLUMNS,"quality"])
    if not cfg.get("required_quality"):
        raise InputError("声明required_quality")
    scale = 0.001 if cfg["input_length_unit"] == "mm" else 1
    for row in tool_rows+base_rows:
        if row["quality"] != cfg["required_quality"]:
            raise InputError("存在不合格的靶标观测，不会静默丢弃")
    def matrix(row):
        out = pose_from_row(row)
        out[:3,3] *= scale
        return out
    output, audit, ids = [], [], set()
    for row in tool_rows:
        if not row["sample_id"] or row["sample_id"] in ids:
            raise InputError("末端靶标sample_id为空或重复")
        ids.add(row["sample_id"])
        t = number(row["sample_time"])
        base = min(base_rows,key=lambda r:abs(number(r["sample_time"])-t))
        delta = number(base["sample_time"])-t
        if abs(delta)>tolerance:
            raise InputError("两靶标超出时间容差")
        pose = tb @ np.linalg.inv(matrix(base)) @ matrix(row) @ np.linalg.inv(te)
        q = Rotation.from_matrix(pose[:3,:3]).as_quat()
        output.append({"sample_id":row["sample_id"],"sample_time":t,
            **dict(zip(("x","y","z"),pose[:3,3])),**dict(zip(("qx","qy","qz","qw"),q)),"quality":cfg["required_quality"]})
        audit.append({"sample_id":row["sample_id"],"base_sample_id":base["sample_id"],"delta_s":delta})
    dest = relative_file(project,cfg["output_data"])
    meta_dest = relative_file(project,cfg["output_manifest"])
    if dest.exists() and len(dest.read_text(encoding="utf-8").splitlines())>1:
        raise InputError("坐标转换输出已存在，选择新版本")
    write_csv(dest,output)
    save(meta_dest,{**tm,"dataset_id":tm["dataset_id"]+"-base-tool","observed_frame":cfg["observed_frame"],
        "source":tm["source"]+"；双靶标安装坐标转换", "sha256":sha256(dest),
        "input_length_unit":"m","installation_reference":cfg["installation_reference"],
        "units":{"length":"m","angle":"rad","time":"s","effort":"N_or_Nm"},
        "conversion_inputs":[{"path":str(p.relative_to(project)),"sha256":sha256(p)} for p in paths+[config_path]]})
    write_csv(dest.with_name(dest.stem+"_frames_audit.csv"),audit)
    print("坐标转换完成：",dest,"。后续pairing中pose_length_unit填写m。")
    return 0


def offset_feedback(project, rev, config_path):
    from core.fieldwork import relative_file
    nominal, fit_path, check_path, dest, fit = checked_fit(project,rev)
    cfg = load(config_path)
    data = relative_file(project,cfg["data"])
    meta_path = relative_file(project,cfg["manifest"])
    rows, meta = dataset(data,meta_path,["joint","q"])
    expected = fit["training_dataset"].get("coordinates_id")
    if not expected or cfg.get("fit_input_coordinates_id") != expected or meta.get("coordinates_id") != expected:
        raise InputError("输入坐标必须与本次运动学拟合CSV的coordinates_id一致；不能猜测映射/零位约定")
    if meta.get("kinematic_offsets_applied") or any("q_fit_input" in r for r in rows):
        raise InputError("此数据已经应用运动学零偏，拒绝重复")
    offsets = load(dest/"joint_offsets.yaml")["offsets"]
    app = load(dest/"APPLICATION.json")
    if sha256(dest/"robot.urdf") != app["corrected_urdf_sha256"] or sha256(dest/"joint_offsets.yaml") != app["joint_offsets_sha256"]:
        raise InputError("修正文件已改变")
    model = Model(dest/"robot.urdf")
    output = []
    for row in rows:
        name = row["joint"]
        if name not in model.independent:
            raise InputError("反馈含未定义独立关节")
        q = number(row["q"])+number(offsets.get(name,0))
        element = model.joints[name]
        if element.get("type") != "continuous":
            limit = element.find("limit")
            if not number(limit.get("lower")) <= q <= number(limit.get("upper")):
                raise InputError("附加零偏后超出确认运行范围；未截断或扩大范围")
        output.append({**row,"q_fit_input":row["q"],"q":q,"kinematic_offset":offsets.get(name,0)})
    out = relative_file(project,cfg["output_data"])
    manifest = relative_file(project,cfg["output_manifest"])
    if out.exists() or manifest.exists():
        raise InputError("输出已存在，请选择新版本")
    kind = meta["data_kind"]
    if fit["training_dataset"]["data_kind"] != "real" and kind == "real":
        kind = "simulation"
    write_csv(out,output)
    save(manifest,{**meta,"dataset_id":meta["dataset_id"]+"-kinematic-offsets", "sha256":sha256(out),
        "data_kind":kind,"coordinates_id":"urdf:"+sha256(dest/"robot.urdf")+":with-kinematic-offsets",
        "kinematic_offsets_applied":True,"calibration_sha256":sha256(fit_path),"input_coordinates_id":expected,
        "is_real_robot_acceptance":False,"hardware_control_commands_sent":False})
    print("附加零偏后的只读记录：",out,"。真机逆映射需厂家控制适配器另行验证。")
    return 0


def self_test(destination, root):
    if destination.exists() and any(destination.iterdir()):
        raise InputError("自检输出目录必须为空")
    destination.mkdir(parents=True, exist_ok=True)
    from examples.run_offline_demo import run_demo
    from examples.run_joint_reference_demo import run as reference_demo
    from core.joint_mapping import JointMapping
    run_demo(destination/"synthetic")
    data = destination/"synthetic"
    reference_demo(data/"nominal.urdf", "j1", destination/"reference")
    # Use the same field application operations the site will use.
    project = destination/"application"
    (project/"parameters").mkdir(parents=True)
    shutil.copytree(root/"configs", project/"configs")
    shutil.copy2(data/"parameters.xlsx", project/"parameters/robot_parameters.xlsx")
    shutil.copy2(data/"mechanisms.yaml", project/"configs/mechanisms.yaml")
    shutil.copy2(data/"acceptance.yaml", project/"configs/acceptance.yaml")
    save(project/"project.yaml", {"baseline": "../synthetic/nominal.urdf"})
    from core.cli import main
    r = project/"results/TEST"
    r.mkdir(parents=True)
    nominal = project/"models/nominal_TEST/robot.urdf"
    commands = [
        ["build-urdf", "--parameters", project/"parameters/robot_parameters.xlsx", "--baseline", data/"nominal.urdf", "--mechanisms", project/"configs/mechanisms.yaml", "--destination", nominal.parent, "--output", r/"nominal.json"],
        ["calibrate-kinematics", "--data", data/"kinematics_calibration.csv", "--manifest", data/"kinematics_calibration.yaml", "--urdf", nominal, "--config", data/"calibration.yaml", "--output", r/"fit.json"],
        ["validate-kinematics", "--data", data/"kinematics_validation.csv", "--manifest", data/"kinematics_validation.yaml", "--urdf", nominal, "--calibration", r/"fit.json", "--acceptance", data/"acceptance.yaml", "--output", r/"holdout.json"],
    ]
    for args in commands:
        if main(list(map(str,args))) != 0:
            raise InputError("应用链路自检失败")
    apply_kinematics(project, "TEST")
    verify_applied(project, "TEST")
    # Move the entire evidence tree and verify without modifying a report/hash.
    moved = destination/"改名后 带空格"
    moved.mkdir()
    for name in ("synthetic", "reference", "application"):
        shutil.move(str(destination/name), str(moved/name))
    JointMapping(moved/"reference/mapping.yaml", moved/"synthetic/nominal.urdf", allow_candidate=True)
    verify_applied(moved/"application", "TEST")
    save(destination/"self_test.json", {"status": "synthetic_pass", "offline_core_workflow": True,
        "relocated_evidence_verified": True, "applied_urdf_verified": True,
        "hardware_commands_sent": False, "is_real_robot_acceptance": False})
    print("自检完成：全部为合成数据；已验证换路径后的证据和修正模型。")
    return 0
