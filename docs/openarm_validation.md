# OpenArm 接入与程序验证

本例使用本地 OpenArm V2 双臂模型，运行工具包自己的程序。厂家原始工程只读；生成模型、网格、记录和报告均写入指定的新目录。模型文件由调用者提供，仓库不捆绑本机模型或日志。

## 官方依据

- [OpenArm Robot Description](https://docs.openarm.dev/api-reference/description/)：名义机构、关节与 mimic 定义。
- [OpenArm CAN 源码](https://github.com/enactic/openarm_can/blob/a30364622ca939b8c8a741167c317b3e95e38a49/src/openarm/damiao_motor/dm_motor_control.cpp)：`create_refresh_command` 状态查询包。
- [NVIDIA Isaac Sim 6.1.0 Prims](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/extensions/isaacsim.core.experimental.prims/docs/index.html)：Articulation 的仿真关节状态设置、RigidPrim 的 tensor 位姿读回。

这些操作不提供新硬件的通用使能、回零或机械端点测量规程。

## 单独检查连接

```bash
python run.py check-openarm --interfaces can0 can1 \
  --raw-frames outputs/openarm/check01_raw.json --output outputs/openarm/check01.json
```

默认仅接收。显式增加 `--query-state` 才发送有界的 `0xCC` 状态查询，每路只查询已部署的 ID 1–8 一轮，不发送使能、失能、模式、参数或运动目标。查询要求接口健康、CAN-FD 1M/5M，且不存在已运行的 ros2_control_node。条件不满足时保留 blocked；不重置接口、不扫描其他波特率。

若接口处于 ERROR-WARNING，可显式增加 `--probe-warning`，做一轮有界状态查询；一旦进入 ERROR-PASSIVE 或 BUS-OFF 就停止。此选项不会重配接口；在本次现场测试中，成功查询后两路错误计数下降并回到 ERROR-ACTIVE。状态和统计字段依据 [Linux SocketCAN 文档](https://docs.kernel.org/networking/can.html)。

ready 仅表示当前期望 ID 有完整回包，左右物理对应、反馈换算和标定仍需验证。原始反馈不会被当成已校准关节角输出。USB CAN 接口为 UP 并不能单独证明电机连通。

## 完整模型与 Isaac 检查

```bash
python examples/run_openarm_validation.py \
  --urdf /实际路径/installed_startup_model.urdf \
  --description-root /实际路径/openarm_description \
  --launcher /实际路径/IsaacSim6.1环境/bin/python \
  --destination outputs/openarm_case01 \
  --check-can --query-state
```

若要同时执行三轮真实电机反馈采集，增加 `--capture-motors --motor-python /安装了openarm_can的环境/bin/python`。仅在当前 CAN 检查 ready 时采集。名义范围检查不通过就保留问题，不生成裁剪后的姿态；通过时可将 14 轴反馈作为仿真状态测试输入，夹爪设为单独标注的 0 参考值。这个步骤只重现臂关节输入，不宣称实际夹爪对齐或独立几何标定。

`description-root` 指向包含 `assets/` 的实际源码包目录。SDK 的 Python 缺依赖时，可将依赖装到独立目录并添加 `--extra-pythonpath`。本例同时需要普通 Python 和 SDK Python 中的工具包依赖。

流程为：

1. `prepare_openarm.py` 冻结原始 URDF 和网格哈希，解析 package URI，构造名义模型参数表。空的固定 world/grasp frame 保存在 sidecar，只从物理导入模型排除；不人为赋予质量。这里只支持 identity world 挂载和空固定叶子 frame。
2. 执行结构检查、URDF 重建、Isaac 导入及静态一致性检查。参数表中的 confirmed 表示已核对名义文件的复制值；source 与 validation_scope 明确注明软件测试，不表示经过真机测量。
3. 在零重力、无驱动、无自碰撞的专用仿真场景中，对独立关节施加测试姿态。状态设置只作用于 Isaac。比较工具包 FK 与 PhysX tensor 连杆位姿，不读取 USD 文件里静态写入的姿态来冒充仿真结果。
4. 生成 `summary.json`、逐连杆误差 CSV，以及 `report/report.html`。CAN 检查失败不会冒充成功，也不阻止独立的软件检查完成。

测试配置中的中位角、±0.05 rad 和数值误差阈值仅用于仿真程序测试，不是实机运动规程、限位裕量或硬件精度要求。没有配置真实 PD、摩擦或接触测试；名义模型成功导入不表示动力学已经标定。

## 当前验证范围

2026-11-02 本机名义模型：原始 24 links / 23 joints；物理导入模型 21 links / 20 joints；16 个独立关节包含双臂 14 轴和 2 个夹爪驱动，另有 2 个 mimic 从动关节。

33 组姿态 × 21 个连杆 = 693 次软件位姿比较通过；另外三组臂反馈输入也完成仿真重现与位姿检查。机器可读数值记录见 [OpenArm 验证摘要](verification/openarm_20261102_summary.json)。这支持当前模型下的程序链路正确性，不支持“真实机器人达到亚微米精度”等结论。

两路最初处于 ERROR-WARNING，默认检查按条件阻止查询。随后使用有界状态查询收到 16/16 电机回包，两路恢复 ERROR-ACTIVE。再通过官方 SDK 完成三轮、48 条真实电机反馈采集。14 个臂关节在“当前驱动配置使用电机角直传”的假设下位于名义范围；夹爪传动/方向单独待核对。没有实机运动、限位搜索或 PD 辨识。

ROS Domain 42 当时没有 `/joint_states`，因此本次采集直接使用官方 codec，不启动会使能和回零的硬件控制器。`OpenArmReadOnlyAdapter` 只调用 SDK 的 Motor、CanPacketEncoder 和 CanPacketDecoder，不创建 OpenArm 控制器对象。采样保留 `motor_position_rad` 与原始帧，不写成已校准的 `q:关节`，SDK 扭矩也标明未标定。

可通过 `examples.openarm_motor_profile.motor_profile()` 创建显式配置，再调用已有 `capture-data`；使用安装了官方 openarm_can Python 绑定的解释器。该 profile 来自本机部署默认值，不能代替其他硬件型号的映射确认。`examples/analyze_openarm_feedback.py` 按明确假设检查名义范围，不自动改符号、零位或限位。
