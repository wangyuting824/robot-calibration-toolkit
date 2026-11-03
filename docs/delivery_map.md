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
| validate-joint-reference | D01、D02 配套、D05 | 独立参考验证、带模型与来源绑定的映射配置 |
| map-joint-feedback | D04、D05 | 保留原始列的坐标换算数据、manifest 与拒绝记录 |
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

现场统一入口 `field.py` 的新增操作如下。全部是项目代码，测量定义与适用边界见[现场操作手册](现场操作手册.html)。

| 操作 | 交付对象 | 输出 |
|---|---|---|
| doctor / init / status | D01、D04、D05 | 环境、相对路径项目、参数/采样模板、未完成记录 |
| edit / seal | D01、D04、D05 | 可追溯参数备份、改动记录、数据哈希 |
| transform-poses / pair-poses | D04、D05 | 双靶标坐标换算、显式时间/单位配对、质量拒绝与审计 |
| run | 原CLI对应交付 | 不覆盖的版本任务、结果与日志 |
| apply / verify-applied | D02、D03配置、D05 | 修正URDF、单独offset、基座/工具坐标、输出模型复验 |
| offset-feedback | D02配套、D04、D05 | 附加零偏的只读记录，不改硬件 |
| apply-drives | D03候选、D05 | 明确力驱动模型的候选增益配置，仍需独立响应验收 |
| export / backup / build-bundle / check-bundle | D01—D06 | 分类归档、完整项目、可搬迁运行环境及完整性校验 |
| self-test | D04、D05 | 合成链路与证据搬迁检查，不代表真机精度 |
