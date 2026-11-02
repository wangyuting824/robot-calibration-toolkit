# 操作流程

先阅读 [来源及边界](sources.md)。以下命令在工具包根目录执行，路径可替换。基础计算使用普通 Python 环境；涉及 SimulationApp 的命令使用安装的 Isaac Sim Python 启动器。

## 1. 软件自检

```bash
python -m pip install -e '.[test]'
python run.py preflight
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
python examples/run_offline_demo.py --destination /tmp/robotcal-demo-new
```

## 2. 原始资料与参数

按 README 的 prepare-inputs 命令生成参数主表，核对各行来源和状态。未知值留空；数字 0 代表真实零值。结构确认不自动确认惯性，links 的 inertial_status/inertial_source 单独填写。

`configs/mechanisms.yaml` 必须明确是否存在闭环、联动，以及对应资料。生成非草稿 URDF 时需要该文件已确认。

```bash
python run.py validate-structure --parameters inputs/confirmed_parameters/robot_parameters.xlsx \
  --mechanisms configs/mechanisms.yaml --require-inertia
```

错误会产生 failed，待确认项会产生 pending。问题解决后才生成正式模型；草稿预览需显式指定 --draft。

## 3. 关节参考与限位

按 [数据说明](measurement_guide.md)准备 SI 单位记录与 manifest，不能把命令角度当独立测量。

```bash
python run.py calibrate-joint-reference --data inputs/measurements/joint_reference/data.csv \
  --manifest inputs/measurements/joint_reference/manifest.yaml --f-scale 填写参考测量单位下的损失尺度
python run.py verify-joint-limits --data inputs/measurements/joint_limits/data.csv \
  --manifest inputs/measurements/joint_limits/manifest.yaml --specification configs/limits.yaml
```

其中 f-scale 是数值拟合设置，不是厂家规定的测量精度。限位程序只处理提供的记录，不向机器人发出搜索机械止挡的动作。

## 4. 运动学拟合与独立验证

在 configs/calibration.yaml 中填写观测坐标、末端 link、基座与工具初值、测量不确定度和待拟合参数边界。

```bash
python run.py calibrate-kinematics --urdf deliverables/D02_urdf/robot.urdf \
  --data inputs/measurements/kinematics/training.csv \
  --manifest inputs/measurements/kinematics/training.yaml \
  --config configs/calibration.yaml --output outputs/kinematics_fit.json
python run.py validate-kinematics --urdf deliverables/D02_urdf/robot.urdf \
  --data inputs/measurements/kinematics/validation.csv \
  --manifest inputs/measurements/kinematics/validation.yaml \
  --calibration outputs/kinematics_fit.json --acceptance configs/acceptance.yaml \
  --errors-csv outputs/kinematics_errors.csv
```

验证必须使用拟合时的同一份名义 URDF；程序在计算时施加修正，避免重复施加。重建 URDF 时使用 build-urdf --calibration；关节零偏另存到 joint_offsets.yaml，不能同时再把同一零偏烘焙进 origin 和驱动反馈。

基座/工具修正随 calibration.yaml 交付，由场景/TF 使用；本程序不静默改变世界坐标。

## 5. Isaac Sim

填写 fix_base、物理步长、驱动、接触和闭环实现。build-isaac-asset 要求 D02 中的 model_manifest.yaml 与 URDF 哈希一致，并且结构和惯性分别已确认。

```bash
/实际/IsaacSim/python.sh run.py build-isaac-asset \
  --urdf deliverables/D02_urdf/robot.urdf --config configs/isaac.yaml \
  --destination deliverables/D03_isaac --output outputs/isaac_import.json
/实际/IsaacSim/python.sh run.py validate-isaac-asset \
  --urdf deliverables/D02_urdf/robot.urdf --stage deliverables/D03_isaac/scene.usda \
  --acceptance configs/acceptance.yaml
```

使用 **6.1.0** 的 Python 启动器。导入和运行时检查 `isaacsim.core.version.get_version()`，与 `expected_isaac_sim_version` 匹配才执行。默认要求 6.1.0，不自动退回 5.1。已有 5.1 的兼容测试必须显式改配置，或给示例增加 `--expected-version 5.1.0`；不能作为 6.1.0 测试结果。

可用下面的可选测试验证实际 SDK，仅使用合成双关节模型：

```bash
python examples/run_isaac_smoke.py --launcher /实际/IsaacSim/python.sh \
  --destination /tmp/robotcal-sdk-new
```

SDK 的 Python 也需要 numpy、scipy、PyYAML、openpyxl。可将缺少依赖安装到独立目录，并通过 --extra-pythonpath 指定，不必修改其他工程的环境。

若采用 pip 安装版，`--launcher` 可以直接指向该虚拟环境的 `bin/python`，无需独立安装版的 python.sh。不要混用其他版本的 ISAAC_PATH、EXP_PATH 或 CARB_APP_PATH；SDK 示例会为子进程清除这些继承值。输入模型和输出场景均使用本工具包自己的目录。

## 6. 官方系统辨识入口

按 [6.1.0 官方文档](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/robot_setup/ext_isaacsim_robot_setup_sysid.html)准备遥测与 stage，填写 configs/nvidia_sysid.yaml。官方脚本位于已安装 SDK 的 `exts/isaacsim.robot_setup.sysid/tools/headless_sysid_solve.py`；入口检查其 SDK VERSION 并记录脚本哈希。工具包不猜测遥测格式。

recipe 需按实际工况选取。manipulator、drive_calibration_inair 等的物理后端、控制器及参数范围不同，参照官方说明并通过 Check，不能一律套用。此入口执行已安装的官方工具，项目本地拟合是另一套有明确适用条件的计算。

```bash
python run.py run-nvidia-sysid --config configs/nvidia_sysid.yaml
python run.py run-nvidia-sysid --config configs/nvidia_sysid.yaml --execute
```

第一条产生调用计划；第二条执行官方工具。官方日志中的 Check、留出集和结果解释仍需审查。进程正常退出不等于机器人验收通过。旧 SDK 没有该扩展时保留 blocked 状态。

## 7. 报告及交付

```bash
python run.py generate-report --results outputs --destination deliverables/D05_reports
python run.py package-delivery --config configs/delivery.yaml \
  --destination deliverables/robot-calibration-draft-v1.zip
```

configs/delivery.yaml 由项目填写，明确六类资料及验收报告路径。追加 --final 会核对所需真实验证、指标范围和当前文件哈希；合成示例不能通过这个门禁。
