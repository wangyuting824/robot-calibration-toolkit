# 反馈与 URDF 不对齐时，如何修正

按同一条流程处理：**保留原始记录 → 核对坐标定义 → 拟合换算 → 独立样本验证 → 导出配置 → 只读换算 → 仿真复验**。参数依据充分后才用于该台机器的反馈转换。程序不改变电机零点、运行限位或控制器。

## 先确定差异属于哪一类

| 差异 | 处理位置 | 对应程序 |
|---|---|---|
| 名称、单位、方向、零偏或线性传动比例 | 原始反馈到 URDF 关节坐标的换算层 | `calibrate-joint-reference` → 新增 `validate-joint-reference` → 新增 `map-joint-feedback` |
| 关节轴线、连杆坐标或几何定义错误 | 已确认参数表与模型 | `validate-structure`、`build-urdf`；选定原点修正可用运动学拟合，轴线需有独立依据 |
| 基座、工具坐标或运动学偏差 | 场景/TF、选定的运动学参数 | `calibrate-kinematics`、`validate-kinematics`，再按结果重建模型 |
| 限位定义不一致 | 厂家定义与合规测量记录 | `verify-joint-limits` 核验后更新参数表；不自动探测止挡 |

线性换算为 `q_model = scale × q_source + offset`。适用时速度为 `dq_model = scale × dq_source`。方向可以包含在 scale 的符号中。非线性夹爪需要经验证的专用模型，不能用任意线性比例替代。

夹爪开口宽度（m/mm）、手指关节角（rad）和电机转角是不同量。测得宽度不能直接填成 URDF 转动关节的参考角；需要明确测量点和可验证的几何关系。

## 现有代码完成了什么

- `calibrate-joint-reference`：从独立参考配对数据拟合比例和零偏，要求至少三个不同位置；输出拟合结果与训练范围。
- `validate-joint-reference`：使用不同 sample_id、不同文件的参考数据，核对机器人身份、反馈坐标版本、参考定义、单位及 URDF 哈希，按配置的误差阈值验证。通过后导出映射 YAML；不通过不导出。
- `map-joint-feedback`：按映射转换已保存的数据，原始 q/dq/effort 分别保存在 `q_source`、`dq_source`、`effort_source`；不覆盖输入文件，不裁剪越界值，不外推，不重复应用修正。
- `MappedJointStateAdapter`：把同样的换算接到 ROS 只读订阅或 CSV 回放。它没有运动接口，不能接管驱动。
- 已验证的 profile 绑定 URDF、数据源坐标定义、机器人身份、系数及验证报告哈希；源证据被修改后拒绝使用。

`candidate` 和用 synthetic/simulation 参考生成的 profile，默认拒绝应用。显式 `--allow-candidate` 只允许仿真分析，输出按 synthetic/simulation 标记；不会升级为真机标定。只有真实独立参考验证通过，所选映射项才标记已验证；未换算关节继续保持未验证，整机标定仍需其他交付检查。

力矩不会随位置比例自动换算。映射关节的输出 effort 留空，原发布值保存在 source 列。对本地驱动未实现的速度/力矩字段，可在 ROS 适配器中配置 `unavailable_velocity_joints`、`unavailable_effort_joints`：输出为空，发布者占位零保存在 `dq_published`、`effort_published`，不当作实测值。

## 准备数据并执行

使用 [拟合集模板](../templates/joint_reference_calibration.csv)、[验证集模板](../templates/joint_reference_validation.csv)、[manifest 模板](../templates/joint_reference_manifest.yaml) 和 [阈值配置](../configs/joint_reference_acceptance.yaml)。

CSV 每行：`sample_id,joint,raw,reference`。raw 是原发布坐标，reference 是独立参考下的关节坐标；不要使用控制目标、仿真值或另一份相同编码器反馈充当实测参考。拟合与验证数据应分别采集、保留不同标识。

manifest 需填写机器人身份、发布坐标定义、模型 SHA-256、测量方法与参考零位/正方向、参考单位、数据来源、时间及文件 SHA-256。真实数据的 reference_kind 使用 `independent_measurement` 或 `manufacturer_calibration`；已知答案演示使用 `synthetic_ground_truth`。

```bash
python run.py calibrate-joint-reference \
  --data calibration.csv --manifest calibration.yaml \
  --f-scale 填写参考单位下的数值损失尺度 --output fit.json

python run.py validate-joint-reference \
  --data validation.csv --manifest validation.yaml \
  --calibration fit.json --acceptance acceptance.yaml \
  --urdf robot.urdf --mapping-output mapping.yaml --output validation_result.json

python run.py map-joint-feedback \
  --data raw_joint_states.csv --manifest raw_joint_states.yaml \
  --mapping mapping.yaml --urdf robot.urdf \
  --output-data mapped.csv --output-manifest mapped.yaml --output mapping_result.json
```

验收配置的每个关节填写 `target_joint`、`max_abs_error` 和 `minimum_validation_samples`。单位随 reference_unit；至少三个不同位置是本程序的数值验证门槛，不是厂家通用测量规程。误差阈值应由任务要求和参考工具能力确定，程序没有默认实机精度。训练覆盖范围也不等于整个范围的误差上界已验证。

原始反馈 manifest 另需 `coordinates_id`、`robot_id` 和逐关节 `joint_units`，必须与 profile 的来源一致。模型版本变化、来源不匹配、参考与训练重用、外推、越界或重复换算都会拒绝执行。

只读适配器的配置示例：

```yaml
class: adapters.mapped_joint_state:MappedJointStateAdapter
options:
  urdf: robot.urdf
  mapping: mapping.yaml
  allow_candidate: false
  source:
    class: adapters.ros2_joint_states:Ros2JointStateAdapter
    options:
      topic: /joint_states
      messages: 30
      timeout_s: 10
      expected_joints: [填写所有应收到的关节名]
```

使用现有 `capture-data` 运行上述配置。仿真复验使用 [MuJoCo 流程](mujoco_validation.md) 的 `--joint-states mapped.csv --joint-manifest mapped.yaml`，检查全部已换算关节；有独立参考前不声明物理对齐完成。关节换算只应用一次；`build-urdf` 的运动学校准输入不是本流程的比例/零偏结果，不能混用两类 JSON。

## 不接设备的完整演示

```bash
python examples/run_joint_reference_demo.py \
  --urdf /模型路径/robot.urdf --joint 目标关节名 \
  --destination /tmp/joint-reference-demo-new
```

示例生成已知答案，实际调用拟合、留出验证、配置导出、文件换算和只读回放。示例中的负比例是合成答案，不是对实机夹爪的建议。

## 旧测量记录怎么使用

`examples/audit_openarm_gripper.py` 可核对现有 OpenArm 卡尺测量记录、当前反馈和 URDF。它保留张开/闭合及独立检查点的原分组，复算误差，比较恒等和反号两种假设；不删除坏点、重新标记测量阶段或把宽度变成参考角度。跨会话的机器人身份、零位和发布坐标版本仍需确认。

## 对应交付

| 交付对象 | 本流程新增内容 |
|---|---|
| D01 参数 | 比例、零偏、适用范围、身份/坐标定义与参考依据 |
| D02 模型配套配置 | 与指定 URDF 哈希绑定的 mapping.yaml；不重复烘焙零偏 |
| D04 程序与数据 | 只读适配器、原始配对与反馈、换算后的数据及 manifests |
| D05 报告 | 独立参考验证、换算前后检查与拒绝原因 |
| D06 使用与复现 | 本说明、运行命令、来源和限制 |

D03 的 Isaac Sim 6.1.0 配置与动力学对齐要求继续保留。

## 方法来源

本流程是项目实现，参考官方格式和换算定义；物理参考采集仍遵循实际机器人的厂家规程。

- [ros2_control 官方 SimpleTransmission：方向、传动比例与位置零偏](https://github.com/ros-controls/ros2_control/blob/jazzy/transmission_interface/include/transmission_interface/simple_transmission.hpp)
- [ROS 2 官方 JointState：位置、速度、力矩和可选字段](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/JointState.msg)
- [SciPy 官方 least_squares](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html)
- [ROS 社区 robot_calibration 维护者文档](https://github.com/mikeferguson/robot_calibration)
