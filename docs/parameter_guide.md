# 参数主表填写

统一使用 m、rad、kg、s。URDF origin 的 xyz/rpy 表达关节零配置坐标在父 link 坐标中的变换；axis 表达在关节坐标中，不直接填写未变换的 STEP 世界轴向。

## 主表

- robot_info：填写 name、units，units 固定写 m rad kg s；另填型号、版本、日期。
- links：每个刚体一行，name 唯一。mass 使用 kg；com_xyz/com_rpy 描述惯性坐标；inertia 按 ixx iyy izz ixy ixz iyz，单位 kg·m²。
- joints：每个树边一行，name 唯一，k_id 对应原清单。origin_xyz/origin_rpy、axis 使用三个空格分隔数值。固定关节无需填写限位和轴向；可动关节填写实际类型与允许范围。
- geometry：target_link 是新 link，source_link 是基线 URDF 中的 link；origin 表示 T_target_source。一个刚体可引用多行部件几何，不猜测组合关系。
- frames：记录父坐标及变换，确认观测与工具参考。
- actuators：记录控制器编号、编码器位置和反馈单位。单位换算与零偏分别记录，避免重复除减速比。
- mechanisms：记录联动、被动和闭环，并在 configs/mechanisms.yaml 中给出具体实现引用。
- contacts：记录真实接触对象、实验工况和来源，不能只填一个无来源的摩擦系数。
- sources：记录图纸、参数表、测量设备、规程及版本。

## 确认状态

status=confirmed 表示该行结构/定义已核对，source 必须非空。质量惯性另用 inertial_status=confirmed 和 inertial_source。不能只因为结构正确，就将 CAD 之外填入的测试质量惯性视为实测参数。

工作簿不接受公式单元格作为实测值。缺少的值保持空，程序报告缺项；0 仅代表已确认的零值。

## 机构限制

支持 URDF fixed、revolute、continuous、prismatic，以及线性 mimic。mimic 参数需明确 leader、multiplier、offset。浮动基座或任意闭环不由固定基座 RNEA 处理。

闭环的实际约束需要来自确认后的机构定义，当前导入工具可叠加经过工程验证的 USD 约束层。未给出实现时会阻止物理模型构建，不能把闭环部件随意固定或假装拆环后物理等价。
