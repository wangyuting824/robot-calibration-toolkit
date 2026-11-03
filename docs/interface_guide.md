# 机器人与设备适配

相同格式、相同可观测参数可以共用程序。轴数、几何、限位和惯性通常放入配置；SDK、通信协议和传感器格式通过适配器接入；机构类型和测量条件变化时需选择匹配求解模型。

## 已实现接口

- ReplayAdapter：读取已存在 CSV。回放数据不代表对新命令产生的物理响应。
- IsaacAdapter：默认检查并调用 NVIDIA 6.1.0 文档 API，只发送仿真位置目标并读取位置/速度。报告记录实际版本和构建信息。不会把广义关节约束力当作已分离的电机驱动力矩输出。
- OpenArmReadOnlyAdapter：调用已安装官方 codec 做状态查询与原始电机反馈采集；无运动接口，左右/零位未经独立确认时保留为 motor telemetry。详见 [OpenArm 验证](openarm_validation.md)。
- Ros2JointStateAdapter：只订阅已有控制器的 JointState，保留发布者与接收时间、原始反馈值以及空缺的可选字段；不创建命令发布者。配置、换算边界与 MuJoCo 核对见 [MuJoCo 测试说明](mujoco_validation.md)。

## 厂家 SDK 适配器

RobotAdapter 和 MetrologyAdapter 是待接入接口。它们在未实现时明确抛出 InputError，不能假装已读取或执行硬件。

真实机器人执行适配器需要：

1. 根据厂家 SDK 实现 read_state、command、stop、close。
2. capabilities 明确声明 read_state、command、watchdog、stop。
3. 配置 manufacturer_procedure、commissioning_reference、limits_confirmed 和起点容差。
4. 保证反馈与命令使用同一套 SI 关节坐标，正确应用反馈换算和零偏，命令执行逆换算。
5. 完成设备特定的状态有效性、看门狗、通信中断和停止行为调试。

run-test --target real 需要显式 --execute，并先检查轨迹、速度和实测起点。该通用程序不是安全控制器，也不承担厂家安全功能。当前工具包不会通过自动换 SDK、猜 CAN 地址或扩大限位来绕过缺失接口。

测量设备需要输出时间戳、坐标、单位、角度/位姿及测量质量，保留设备校准依据。多设备时钟同步由实际同步方案确认，不能仅凭两个 CSV 的行号配对。
