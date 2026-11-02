# 程序与交付对象

全部程序源码属于 D04；以下列出主要业务输出。

| 程序 | 对应交付对象 | 主要输出 |
|---|---|---|
| preflight | D05 | 环境与资料缺项 |
| check-openarm | D04、D05 | CAN 接收、可选非运动查询及连接缺项 |
| prepare-inputs | D01、D04 | 未确认参数表与模型基线 |
| validate-structure | D01、D05 | 结构、单位及参数检查 |
| build-urdf | D02 | URDF、网格、碰撞、标定与模型清单 |
| capture-data | D04 | 数据和带来源的 manifest |
| run-test | D04、D05 | 明确目标下的运行日志 |
| calibrate-joint-reference | D01、D05 | 反馈换算与零位拟合 |
| verify-joint-limits | D01、D05 | 限位记录核验与重复性 |
| calibrate-kinematics | D01，供 D02 重建 | 选定关节及坐标修正 |
| validate-kinematics | D05 | 独立位置与方向验证 |
| build-isaac-asset | D03 | 实际 NVIDIA 导入资产与场景 |
| validate-isaac-asset | D05 | USD/URDF 静态一致性 |
| validate-runtime-kinematics | D05 | 工具包 FK 与实际 Isaac 连杆位姿数值对比；仅仿真 |
| run-nvidia-sysid | D03、D05 | NVIDIA 6.1.0 官方辨识调用计划、脚本哈希、日志与官方结果 |
| identify-dynamics | D01，供 D03 更新 | 本地附加惯量和摩擦回归 |
| tune-drives | D03、D05 | 指定力驱动模型的候选增益 |
| identify-contact | D01、D03、D05 | 明确接触模型的拟合结果 |
| validate-dynamics | D05 | 匹配工况下的响应比较 |
| generate-report | D05 | HTML、JSON 汇总报告 |
| package-delivery | D01–D06 | 分类归档和 SHA-256 清单 |

D06 来自 docs 中的安装、填写、采集、接口及验收说明。项目文件未包含实机标定数据时，交付的是可使用的软件工具包，不应将其称为已标定机器人的正式模型交付。
