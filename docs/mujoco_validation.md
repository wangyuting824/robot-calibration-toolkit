# 用 OpenArm 检查工具包，并在 MuJoCo 中打开 URDF

这套流程不需要额外测量仪器。OpenArm 用来检查程序和数据链路，旭创项目仍需自己的参数、测量与验收。MuJoCo 是本次新增的模型测试环境，D03 的 Isaac Sim 6.1.0 交付要求继续保留。

现场版会将 ASCII STL 转为二进制 STL；超过本机 MuJoCo STL 解码面数上限的网格转为 OBJ，保留所有三角面并记录来源和转换哈希，不做简化。依据是 [MuJoCo 官方 mesh 格式说明](https://mujoco.readthedocs.io/en/3.17.0/XMLreference.html#asset-mesh)。运行时使用输出目录 `model/robot.urdf`，避免 `mj_saveLastXML` 可读 MJCF 导出的数字舍入；`robot.xml` 仅保留为可读参考。原 URDF 和网格不改动。

## 能检查什么

| 检查 | 输入与做法 | 结果能证明什么 |
|---|---|---|
| 资料和参数 | 读取已有 ZIP、Excel/YAML，检查结构、单位和惯量 | 文件可读、字段与模型定义是否一致 |
| URDF 构建 | 从 OpenArm 当前控制器使用的模型生成名义参数表，再构建 URDF | 同一模型的整理和生成链路可用；名义参数没有因此获得实测确认 |
| 真机状态采集 | 订阅已有 ROS 2 `/joint_states`，保存时间戳、名称、位置和可选速度/力矩 | 状态能进入工具包；不确认发布者的零位、方向、夹爪换算或力矩精度 |
| MuJoCo 导入 | 官方编译器读取 URDF；DAE 视觉网格先转换为 OBJ | 模型能够加载；保留轴线、限位、质量惯性和 mimic 关系 |
| 连杆位姿计算 | 多组关节角输入，比较工具包 FK 与 MuJoCo `mj_forward` | 两套软件对同一模型的计算是否一致；不表示实物位置精度 |
| 仿真步进 | 固定基座、零重力、关闭接触，施加很小的仿真测试力矩并运行 100 步 | 引擎能计算有限响应；不表示控制响应或接触已对齐 |
| 标定算法和交付门禁 | 带已知答案的合成数据完成拟合、独立验证、报告和打包 | 算法和流程能运行；合成数据会被正式真机交付门禁拒绝 |

没有独立参考时，真实零偏、轴线偏差、机械限位、末端绝对精度、连杆惯量、力矩和接触精度仍待验证。静止反馈不能验证真实轨迹跟踪。程序不会自动搜索机械止挡，也不会发送使能、回零或运动目标。

## 安装

```bash
python -m pip install -e '.[test,mujoco]'
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
python examples/run_offline_demo.py --destination /tmp/robotcal-offline-new
```

MuJoCo 测试使用 3.17.0，DAE 转换使用 trimesh 4.12.2 与 pycollada 0.9.3。原始 URDF 和网格不修改。转换副本带源文件与输出文件 SHA-256；保留 URDF 的缩放和镜像。带外部纹理的 DAE/OBJ 会明确拒绝，避免遗漏依赖。MuJoCo 的网格接触几何与其他引擎不保证相同。

## 准备模型并检查

先使用现有 `examples/prepare_openarm.py` 的 `prepare(source_urdf, description_root, destination)` 整理 OpenArm 模型，随后执行 `validate-structure` 与 `build-urdf`，具体过程同 [OpenArm 接入说明](openarm_validation.md)。`description_root` 使用实际的 `openarm_description` 包目录。整理脚本仅为名义模型软件测试移除空的固定坐标帧，并把它们保存在 sidecar；不增加虚构质量。它不把源模型参数变成实测值。

获得可解析本地网格的 URDF 后：

```bash
python examples/run_mujoco_validation.py \
  --urdf /实际路径/built/robot.urdf \
  --destination /tmp/robotcal-mujoco-new

python examples/view_mujoco.py \
  --urdf /实际路径/built/robot.urdf \
  --mjcf /tmp/robotcal-mujoco-new/model/robot.urdf \
  --poses /tmp/robotcal-mujoco-new/view_poses.json
```

每次检查选择新的 destination。查看器中鼠标旋转视角，左右方向键切换姿态，`R` 播放测试姿态。查看器只进行关节状态设置和 FK 刷新；这些动作不是物理控制轨迹，也不会传到真机。

## 从已有 ROS 控制器采集

控制器运行时优先订阅它的状态，避免另开程序接管 CAN。没有运行控制器时，已有 `OpenArmReadOnlyAdapter` 仍可用于限定的状态查询；这两种接口不并行接管硬件。

建立 `interface.yaml`：

```yaml
class: adapters.ros2_joint_states:Ros2JointStateAdapter
options:
  topic: /joint_states
  messages: 30
  timeout_s: 10
  expected_joints: [填写该机器人的所有独立关节名称]
```

另建数据 manifest，包含 `dataset_id`、`role: validation`、`data_kind: real`、SI 单位、时钟和来源；示例格式见 [接口说明](interface_guide.md)。时钟需说明 `t` 是本机接收时间，`stamp_sec/stamp_nanosec` 是发布者时间，硬件同步尚未独立确认。标记 `joint_mapping_verified: false`、`effort_reference_calibrated: false`。

在安装 ROS 的 Python 环境中执行：

```bash
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=42 /usr/bin/python3 run.py capture-data \
  --interface interface.yaml --manifest manifest.yaml \
  --output-data joint_states.csv --output-manifest joint_states.yaml \
  --max-samples 480 --output capture_result.json
```

域号、关节名、消息数量按现场设置。16 个关节 × 30 帧 = 480 行；`max-samples` 应保证整帧收全。适配器检查重复关节、缺失位置、可选数组长度、有限值和递增时间戳；缺失速度/力矩保留为空，不补成零。ROS 消息中的力矩仅按发布者语义保存，不能据此视为已校准力矩。

采集后加 `--joint-states joint_states.csv --joint-manifest joint_states.yaml` 运行 MuJoCo 检查，程序先核对单位、来源、时钟和 SHA-256。若夹爪换算尚未确认，可以显式增加 `--reference-joints openarm_left_finger_joint1 openarm_right_finger_joint1`：仅在仿真中把这两个关节设为零。原始 CSV 和范围异常完整保留；报告列明替换项。其余反馈越界的整帧会被拒绝，不能通过裁剪角度获得“通过”。

还可以用 MuJoCo 生成带已知零偏的合成位姿，检查实际标定程序能否恢复左右各 7 个关节偏移，并使用不同样本作验证：

```bash
python examples/check_openarm_calibration_math.py \
  --urdf /实际路径/built/robot.urdf \
  --mjcf /tmp/robotcal-mujoco-new/model/robot.urdf \
  --destination /tmp/robotcal-openarm-math-new
```

该实验每臂使用 48 个拟合样本和 24 个留出样本，所有数据标记为 synthetic。基座和工具变换在此测试中已知，只拟合每臂 7 个零偏。恢复已知答案证明数值求解在该模型上可用，不代表测得了实机零偏，也不证明任意参数组合都可辨识。

## 结果和依据

输出包括 `model/robot.urdf`、`model/robot.xml`、网格副本与来源清单、`fk_errors.csv`、`summary.json`。有反馈时另有范围检查、反馈 FK 误差和完整采集姿态。查看器从第一组已接收的臂数值开始，后续为名义模型测试姿态。数据与原始模型保留在本地 `outputs/`，不上传 GitHub。

本仓库程序由项目编写。官方依据是格式和 API，并非厂家为本项目发布的完整标定规程：

- [MuJoCo 官方 URDF 支持与扩展](https://mujoco.readthedocs.io/en/3.17.0/modeling.html#urdf-extensions)
- [MuJoCo 编译选项、关节与 equality 约束](https://mujoco.readthedocs.io/en/3.17.0/XMLreference.html)
- [MuJoCo Python 与 passive viewer](https://mujoco.readthedocs.io/en/3.17.0/python.html)
- [ROS 2 官方 JointState 消息定义](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/JointState.msg)
- [ROS 2 官方 rclpy 实现](https://github.com/ros2/rclpy/tree/jazzy/rclpy/rclpy)
- [trimesh DAE 导入实现文档](https://trimesh.org/trimesh.exchange.dae.html)
