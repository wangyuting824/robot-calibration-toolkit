"""Offline field entry point. All project file paths are relative to project root."""
from pathlib import Path
import argparse
import json
import os
import sys

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
if os.environ.get("ROBOTCAL_DISABLE_NETWORK") == "1":
    from core.offline_guard import install
    install()


def parser():
    p = argparse.ArgumentParser(description="现场标定工具：不需要 Codex；先 doctor，再 init/status/run。")
    sub = p.add_subparsers(dest="action")
    d = sub.add_parser("doctor", help="检查本机环境及可选 SDK，不启动机器人")
    d.add_argument("--project")
    d.add_argument("--output")
    i = sub.add_parser("init", help="从厂家 ZIP 建立独立项目；不会确认任何参数")
    i.add_argument("--project", required=True)
    i.add_argument("--archive", required=True)
    for key in ("docx", "step"):
        i.add_argument("--" + key)
    for action in ("status", "run", "seal", "edit", "apply", "verify-applied", "backup", "pair-poses", "transform-poses", "offset-feedback", "apply-drives", "preview", "export"):
        s = sub.add_parser(action)
        s.add_argument("--project", required=True)
        s.add_argument("--revision", default="R01")
        if action == "run":
            s.add_argument("job", help="jobs 列表见 status；参数写 configs/jobs.yaml")
        elif action == "seal":
            s.add_argument("--data", required=True)
            s.add_argument("--manifest", required=True)
        elif action == "edit":
            s.add_argument("--sheet", choices=["links", "joints", "frames", "actuators"], required=True)
            s.add_argument("--name", required=True)
            s.add_argument("--field", required=True)
            s.add_argument("--value", required=True, help="JSON 数字、数组或带双引号的字符串")
            s.add_argument("--source", required=True)
        elif action == "backup":
            s.add_argument("--destination", required=True)
        elif action in ("pair-poses", "transform-poses", "offset-feedback"):
            s.add_argument("--config", default={"pair-poses":"configs/pairing.yaml", "transform-poses":"configs/pose_transform.yaml", "offset-feedback":"configs/offset_feedback.yaml"}[action])
        elif action == "export":
            s.add_argument("--final", action="store_true")
            s.add_argument("--destination", required=True)
        elif action == "preview":
            s.add_argument("--viewer", action="store_true", help="检查通过后打开MuJoCo查看器；仅仿真")
    s = sub.add_parser("self-test", help="全合成链路自检，不能作为真机验收")
    s.add_argument("--destination", required=True)
    b = sub.add_parser("build-bundle", help="已备齐 runtime/wheelhouse 时生成硬盘包")
    b.add_argument("--destination", required=True)
    b.add_argument("--project")
    c = sub.add_parser("check-bundle", help="校验完整工具包 MANIFEST.sha256.json")
    c.add_argument("--root", default=str(ROOT))
    return p


def main(argv=None):
    p = parser()
    a = p.parse_args(argv)
    if not a.action:
        if sys.stdin.isatty():
            try:
                from core.field_console import menu
                return menu(ROOT)
            except ImportError as exc:
                print(f"缺少依赖：{exc}。请使用包含runtime的完整离线包。",file=sys.stderr)
                return 2
        print("现场工具包已启动。无需 Codex。\n先执行：bash 启动工具包.sh doctor\n")
        print("阅读：", ROOT / "docs/现场操作手册.html")
        p.print_help()
        return 0
    try:
        from core.fieldwork import dispatch
        return dispatch(a, ROOT)
    except ImportError as exc:
        print(f"缺少运行依赖：{exc}。请使用含 runtime 的离线包，或先按 docs/现场操作手册.html 安装依赖。", file=sys.stderr)
        return 2
    except Exception as exc:
        print(f"未完成：{exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
