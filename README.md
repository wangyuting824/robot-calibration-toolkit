# 机器人 URDF 标定与 Isaac Sim 对齐工具包

这是为本项目编写的工程工具包。它没有对真实机器人完成标定，也不是 NVIDIA、ROS 或机器人厂家联合发布的软件。

工具包覆盖机构定义检查、URDF 生成、关节参考标定、限位记录核验、运动学拟合与独立验证、Isaac Sim 导入及数据对比。目标仿真版本固定为 **Isaac Sim 6.1.0 / PhysX**。真机 SDK 和测量设备型号确定后，需要完成并调试对应适配器。

## 程序来源与官方依据

| 部分 | 依据 | 本工具包的实现性质 |
|---|---|---|
| URDF、单位和坐标 | [ROS urdfdom](https://github.com/ros/urdfdom)、[REP-103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst) | 独立实现的格式子集、检查和生成工具 |
| 运动学标定 | [robot_calibration 维护者文档](https://github.com/mikeferguson/robot_calibration) | 参考关节零偏、自由坐标和独立观测的思路，自行实现位姿残差优化；不是直接运行该 ROS 包 |
| 数值求解 | [SciPy least_squares](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html) | 调用官方数值库；数值文档不等于物理测量规程 |
| Isaac Sim 导入 | [NVIDIA URDFImporter](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/extensions/isaacsim.asset.importer.urdf/docs/index.html)、[6.x 迁移说明](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/migration_guides/isaac_sim_6_0/urdf_mjcf_importer_exporter_pipeline.html) | 调用官方 API；没有沿用已移除的旧导入命令 |
| 驱动与测试 | [NVIDIA Gain Tuner](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/robot_setup/ext_isaacsim_robot_setup_gain_tuner.html)、[OpenUSD DriveAPI](https://openusd.org/release/api/class_usd_physics_drive_a_p_i.html) | 采用文档中的力驱动 PD 形式；本地增益拟合有严格的数据适用条件 |
| 官方系统辨识 | [NVIDIA 6.1.0 System Identification（Beta）](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/robot_setup/ext_isaacsim_robot_setup_sysid.html) | 调用安装的官方 headless 工具；已核对本机扩展 2.2.3 的脚本接口，实际辨识需要适用遥测与 recipe |
| 本地动力学计算 | [Roy Featherstone 算法作者资料](https://royfeatherstone.org/spatial/v2/) | 独立编写固定基座 RNEA，结合有约束回归；没有捆绑或复制 Spatial_v2 源码 |
| 真机关节限位 | 本机厂家规程必须提供；[Beckhoff 官方资料](https://infosys.beckhoff.com/content/1033/tf5200_homing/343674123.html?id=1788924359209060089)仅说明参考事件与停稳位置的区别 | 核验已有测量记录，不生成未知机械止挡搜索动作 |

完整来源、适用边界和逐程序映射见 [docs/sources.md](docs/sources.md)、[docs/sources.yaml](docs/sources.yaml)。每次运行的 JSON 都附带对应来源。

目录见 [docs/project_layout.md](docs/project_layout.md)，实际 SDK 与离线检查范围、尚未执行的真机验证见 [docs/testing.md](docs/testing.md)。

## 安装与离线自检

基础工具支持 Python 3.10+；Isaac 部分使用 6.1.0 版本文档中的接口，启动后读取官方版本信息并检查是否匹配。只有显式配置 `expected_isaac_sim_version: 5.1.0` 才运行遗留兼容分支，该结果不算作 6.1.0 验证。

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[test]'
python run.py --help
python run.py preflight
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
python examples/run_offline_demo.py --destination /tmp/robotcal-demo
```

离线示例生成带已知答案的 synthetic 数据，运行实际标定/验证程序，并检查正式交付门禁。它不调用真机，不代替 Isaac 物理仿真；示例通过不代表真实机器人精度达标。请为每次演示选择新的输出目录。

## 使用现有机器人资料

现有三份文件放在本目录的上一层，保持其原文件名。仓库只保存路径清单；可修改 `inputs/source_manifest.yaml` 指向实际位置。

```bash
python run.py preflight --sources inputs/source_manifest.yaml
python run.py prepare-inputs \
  --archive ../robot_urdf_external_joints.zip \
  --destination inputs/baseline_model \
  --workbook inputs/confirmed_parameters/robot_parameters_unconfirmed.xlsx
python run.py validate-structure \
  --parameters inputs/confirmed_parameters/robot_parameters_unconfirmed.xlsx \
  --mechanisms configs/mechanisms.yaml
```

生成的主表保留基线数值并明确标为 unconfirmed。±20°、0.5 rad/s、20 N·m 及质量惯性仍属于测试数据。程序不会自动将它们升级为实机参数，也不会把缺少的结构主表当作已恢复。

厂家确认并填好参数、机构表后，生成模型：

```bash
python run.py build-urdf \
  --parameters inputs/confirmed_parameters/robot_parameters.xlsx \
  --mechanisms configs/mechanisms.yaml \
  --baseline inputs/baseline_model/robot_urdf_external_joints/robot.urdf \
  --destination deliverables/D02_urdf \
  --require-inertia
```

仅做未确认模型预览时可显式使用 `--draft`；草稿不能作为已确认物理模型交付。软件只重用已提供的网格，依据明确的分组和坐标重组，不会从 STEP 外观推断轴线。新增零件需要先在 CAD 工具中导出网格并加入基线资产。

## 三阶段操作与交付

1. 机构与关节定义：`prepare-inputs` → `validate-structure` → `build-urdf`。输出 D01 参数资料与 D02 模型。
2. 运动学对齐：`capture-data` → `calibrate-joint-reference` / `verify-joint-limits` → `calibrate-kinematics` → `validate-kinematics`。输出 D01 修正参数、D04 数据和 D05 独立验证报告；再用标定结果重建 D02。
3. Isaac Sim 6.1.0：`build-isaac-asset` → `validate-isaac-asset` → 官方 `run-nvidia-sysid` 或适用的本地拟合 → `run-test` → `validate-dynamics`。输出 D03、D04、D05。

工具清单与 D01–D06 的逐项对应见 [docs/delivery_map.md](docs/delivery_map.md)。详细步骤见 [docs/quickstart.md](docs/quickstart.md)。

## 结果状态

| 状态 | 含义 | CLI 退出码 |
|---|---|---|
| computed / built / ready / generated / packaged | 对应计算、构建或检查完成；不能单凭该状态证明真机已达标 | 0 |
| passed | 当前程序检查范围内、按已配置条件通过；最终交付还检查真实数据与范围 | 0 |
| synthetic_pass / simulation_pass | 模拟数据或仿真数据通过当前检查 | 0 |
| draft / pending / unverified | 草稿、缺少确认或未配置验收条件 | 3 |
| failed / blocked | 检查不通过、输入错误或能力缺失 | 2 |

每次默认产生独立运行结果；可用 `--output 路径.json` 指定名称。输入文件哈希、数据来源、方法依据和状态进入结果。`package-delivery --final` 拒绝以 synthetic 结果作为真机验收依据。

## 当前边界

- 真机、测量设备适配器是明确的待接入接口；设备协议未知时会阻止执行。
- 本地动力学拟合辨识附加关节惯量与摩擦，连杆质量、质心和惯量需有独立依据。
- 力矩不能直接用未标定电流替代；接触回归是显式近似模型，不是厂家通用接触测量规程。
- 闭环机构需要已确认的 USD 约束实现；单纯 URDF 树和 mimic 不能代表任意闭环。
- Isaac 静态一致性检查不等于物理响应通过；还需目标 SDK 中运行及真实实验验证。

依赖及来源说明见 [docs/third_party.md](docs/third_party.md)。
