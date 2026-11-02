# 来源、适用条件与实现归属

核对日期：2026-11-02。导入、Articulation、SimulationManager、Gain Tuner 和 SysID 采用 NVIDIA **6.1.0** 官方文档。已找到本机 6.1.0-rc.26 pip 安装，并核对其 SysID 扩展 2.2.3 的官方脚本。文档是实现依据，实际 SDK 验证范围单独记录于 [testing.md](testing.md)。

机器可读逐程序映射位于 [sources.yaml](sources.yaml)。其中明确区分：

1. 官方 API 包装：调用 NVIDIA URDFImporter、Articulation、SimulationManager。
2. 官方程序调用：run_nvidia_sysid.py 按 6.1.0 文档和安装脚本调用 NVIDIA headless_sysid_solve.py。官方工具自己执行数据质量、可辨识性和优化检查；没有遥测与工况时不会伪造辨识结果。
3. 项目数值实现：运动学残差、RNEA、摩擦回归、PD 回归、接触近似拟合和误差对比。
4. 项目管理流程：资料整理、报告和交付打包。

没有将第 3、4 类标记成 NVIDIA 或机器人厂家的官方算法。SciPy 文档只支持数值求解器的使用，不能据此声称物理测量方法已获厂家认可。

OpenArm 专用的 `check-openarm` 依据已固定版本的 [Enactic 官方状态查询源码](https://github.com/enactic/openarm_can/blob/a30364622ca939b8c8a741167c317b3e95e38a49/src/openarm/damiao_motor/dm_motor_control.cpp)独立封装非运动状态查询；`validate-runtime-kinematics` 调用 NVIDIA tensor 位姿接口并与本地 FK 比较。两者的实际验证范围见 [OpenArm 说明](openarm_validation.md)。

## 运动学

[ROS robot_calibration](https://github.com/mikeferguson/robot_calibration)描述了关节零偏、自由坐标与观测误差的标定。本项目采用独立测量的完整位姿残差，使用 [SciPy bounded least squares](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html) 优化明确选择的参数。

它不是对 ROS 包的逐行移植，不采用未知厂家控制器内部的 DH 标定算法。基座、工具和关节偏差同时释放可能不可辨识；程序检查雅可比秩，但数值满秩仍不能替代实际精度验证。

## Isaac Sim

- [官方导入 API](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/extensions/isaacsim.asset.importer.urdf/docs/index.html)
- [6.x 导入迁移](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/migration_guides/isaac_sim_6_0/urdf_mjcf_importer_exporter_pipeline.html)
- [官方 Articulation](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/extensions/isaacsim.core.experimental.prims/docs/index.html)
- [官方 SimulationManager](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/extensions/isaacsim.core.simulation_manager/docs/index.html)
- [官方 Gain Tuner](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/robot_setup/ext_isaacsim_robot_setup_gain_tuner.html)
- [官方 System Identification（Beta）](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/robot_setup/ext_isaacsim_robot_setup_sysid.html)
- [6.1.0 官方 headless 脚本源码](https://github.com/isaac-sim/IsaacSim/blob/v6.1.0/source/extensions/isaacsim.robot_setup.sysid/tools/headless_sysid_solve.py)
- [6.1.0 官方导入回归测试](https://github.com/isaac-sim/IsaacSim/blob/v6.1.0/source/extensions/isaacsim.asset.importer.urdf/isaacsim/asset/importer/urdf/tests/test_urdf.py)：明确选择资产的 Physics/physx 分支

新的 6.x API 取代已移除的 URDFParseAndImportFile 命令。导入和测试采用实际 SDK；SDK 缺失时不输出伪造 USD 或替代仿真曲线。

[OpenUSD DriveAPI](https://openusd.org/release/api/class_usd_physics_drive_a_p_i.html)规定 USD 角度、角速度和角驱动增益的单位。工具包统一数据为 rad，直接写 USD 角驱动增益时换算为每 degree 的值。Articulation 控制接口仍按其官方定义接收 rad。

## 真机限位与回零

需要本机厂家的资料：正常运行范围、零位定义、驱动反馈含义、测量或调试规程。没有这些资料，工具包不执行机械极限搜索。

[Beckhoff 文档](https://infosys.beckhoff.com/content/1033/tf5200_homing/343674123.html?id=1788924359209060089)仅作为“参考事件锁存与停稳位置不同”的原理说明，不表示本机采用 Beckhoff，也不能把其运动参数或回零策略直接用于本机。

## 动力学与接触

可使用 6.1.0 官方 SysID 的数据格式、预检查和 recipe，也可在满足下面条件时使用本地拟合。不同 recipe 的后端和控制器不同，不能把 Newton 求解器的配置当作 PhysX 已验证配置。

本地 identify_dynamics 使用 [Featherstone 算法作者资料](https://royfeatherstone.org/spatial/v2/)中的空间代数和 RNEA 思路独立实现固定基座刚体树的逆动力学，然后用 [SciPy lsq_linear](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.lsq_linear.html) 拟合附加关节惯量、黏性摩擦、平滑库仑摩擦和偏置。没有复制或分发 Spatial_v2 的 GPL 源码，也不声称能从任意日志识别全部连杆惯性。

本地接触拟合只支持显式选定的滑动摩擦、线性法向柔顺或恢复系数近似。它们属于项目工程模型。需要实验规程、可追溯测力数据及后端映射确认；线性柔顺参数不能直接当作所有 PhysX 接触模型的通用参数。
