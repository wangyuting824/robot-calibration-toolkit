# 测量输入与方法边界

本文件规定程序接受的数据，不提供未知机器人的通用运动试验规程。拼装整机的测量动作、负载、速度、参考位置和边界检查采用本项目建立并确认的规程，依据部件规格、装配约束和测量能力；不要求存在整机厂家。

## 通用 manifest

每个 CSV 配一份 YAML：dataset_id、role（calibration/validation）、data_kind（real/simulation/synthetic）、units、clock、source。单位统一为 {length: m, angle: rad, time: s, effort: N_or_Nm}。可填 sha256 校验文件。

还需要记录设备、参考坐标、工具负载、时钟定义、测量校准证据和规程。原始值和转换方法保留；不能将命令值、名义 TF 或同一 URDF 生成的位姿标作独立物理测量。

## 关节参考

列：joint, raw, reference。reference 是独立参考装置给出的关节角或位移；manifest 填 reference_instrument、raw_unit、reference_unit（rad 或 m）。模型是 reference=scale*raw+offset。它只是换算拟合，量程、装夹、角度溯源和测量精度由设备规程确定。

## 限位记录

列：joint, boundary（lower/upper）, angle。manifest 填 event_kind 和实际 procedure_reference。支持 software_boundary、switch_latch、operating_range_check、manufacturer_stop_record。

规格表按关节填写 lower、upper、kind、source、tolerance、repeatability_peak_to_peak、minimum_repeats。自研整机可用 operating_range_check 核对已经评估的工作范围端点；该记录不证明完整机械行程。程序比较已有记录、重复性与明确规格，不自动制定裕量，不从电流推断止挡，不把停稳位置当成触发沿锁存位置。

关于事件位置与制动后的停稳位置差异，参见 sources 中的 Beckhoff 说明。该参考不能替代本项目的实际机构与控制规程。

## 运动学

每行是一个同步姿态：sample_id, q:关节名（全部独立关节）, x,y,z,qx,qy,qz,qw。四元数顺序 xyzw，必须归一化；观测是已定义工具坐标在 observed_frame 中的完整位姿。配置中的 tip_link 和 tool_xyz_rpy 对应这个工具。

需要独立的 pose_source 和已确认的时钟/坐标关系。训练与验证使用不同 dataset_id 和 sample_id；程序也拒绝同一 CSV 哈希复用。跨会话、跨姿态验证是否足够，由实际验收计划确定，不通过任意固定采样次数宣称精度。

## 动力学

本地固定基座回归需要每个独立关节的 q:名称、dq:名称、ddq:名称、tau:名称。需声明 torque_semantics=joint_actuator_effort、effort_reference 和惯性来源。程序不会直接对带噪位置做二阶差分后冒充可靠加速度。

目标 SDK 为 6.1.0。官方 SysID 入口使用该版本规定的遥测格式及 Check；本工具的 CSV 不自动宣称兼容，需要按官方映射设置导入。

本地模型：tau=RNEA+armature*ddq+b*dq+fc*tanh(dq/v_scale)+bias。固定惯性、固定基座和刚体树是假设条件；输出不是全部连杆惯性的辨识结果。

## 增益与接触

本地 PD 回归需要 joint,q,dq,q_command,dq_command,pd_effort,saturated。pd_effort 必须是已分离的 PD 分量，manifest 声明 isolated_pd_effort 和 force drive。总电机力矩含重力前馈等时不能直接套用。

接触近似模型明确选择一种：滑动 Coulomb+viscous、线性法向柔顺、恢复系数。程序要求实验规程与测力依据；不会从滑动数据推断静摩擦，也不会把线性柔顺参数直接映射到不匹配的刚性接触后端。

## 真机/仿真对比

比较 CSV 为长表：joint,t 加上验收文件 dynamics.required_metrics 中列出的量。默认 q,dq；还支持 tau、normal_force、slip_velocity。接触实体用 joint 列作为统一名称标识，接触专项验证可只要求 normal_force、slip_velocity，不必填写虚构的关节角。

两份 manifest 必须一致声明 test_case_id、实际输入记录的 input_sha256、initial_state、payload_id、time_basis=test_relative。实测时间应按已记录的事件原点统一；程序不拟合平移时间、不做动态时间规整、不外推缺失区间。
