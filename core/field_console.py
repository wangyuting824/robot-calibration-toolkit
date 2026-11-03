"""Small terminal menu for an operator who has no Codex or IDE."""
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace as NS
import json
import subprocess
import webbrowser

from core.fieldwork import doctor, status, run_job, backup
from core.field_validation import apply_kinematics, verify_applied, pair_poses, transform_poses
from core.io import load


def menu(root):
    project = root/"projects/旭创现场项目"
    rev = "R01"
    while True:
        print(f"\n旭创标定现场工具　项目：{project}　版本：{rev}")
        print("1 环境自检　2 项目状态/缺项　3 打开手册　4 运行任务\n5 应用运动学修正　6 复验交付模型　7 靶标坐标转换　8 时间配对\n9 更换项目/版本　B 完整项目备份　0 退出")
        try:
            choice=input("选择：").strip().upper()
            if choice=="0":return 0
            if choice=="1":print(json.dumps(doctor(project),ensure_ascii=False,indent=2))
            elif choice=="2":status(project,rev)
            elif choice=="3":webbrowser.open((root/"docs/现场操作手册.html").as_uri())
            elif choice=="4":
                print("可用任务：", ", ".join(load(project/"configs/jobs.yaml")["jobs"]))
                run_job(project,input("任务名：").strip(),rev,root)
            elif choice=="5":apply_kinematics(project,rev)
            elif choice=="6":verify_applied(project,rev)
            elif choice=="7":transform_poses(project,project/"configs/pose_transform.yaml")
            elif choice=="8":pair_poses(project,project/"configs/pairing.yaml")
            elif choice=="9":
                value=input("项目路径（留空沿用）：").strip()
                if value:project=Path(value).expanduser().resolve()
                value=input("版本如 R02（留空沿用）：").strip()
                if value:
                    from core.fieldwork import revision
                    rev=revision(value)
            elif choice=="B":
                default=project.parent/(project.name+"_"+rev+"_"+datetime.now().strftime("%Y%m%d_%H%M%S")+".zip")
                value=input(f"备份到（留空使用 {default}）：").strip()
                backup(project,Path(value).expanduser().resolve() if value else default)
        except (EOFError, KeyboardInterrupt):
            print("\n退出；已保存结果仍保留。")
            return 0
        except Exception as exc:
            print("未完成：",exc)
