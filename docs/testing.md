# 软件验证记录

日期：2026-11-02。目标 SDK：Isaac Sim 6.1.0，PhysX。

本次合成 SDK 测试的版本、采样数和运动范围见 [机器可读结果](verification/sdk61_smoke_summary.json)。

| 范围 | 本次结果 | 能证明什么 |
|---|---|---|
| Python 自动检查 | 70 passed，Python 3.13 | 已知答案恢复、模型/数值检查、输入拒绝、独立验证门禁、两版静态 USD、版本/超时、OpenArm 非运动查询与反馈语义检查 |
| 离线合成全流程 | 完成 | 结构检查、模型生成、关节换算、限位记录核验、运动学/动力学/驱动/接触拟合、回放、报告、草稿打包可串联 |
| 合成数据正式交付门禁 | 按预期拒绝 | synthetic 不能作为真机正式验收证据 |
| 当前机器人资料处理 | 已整理并生成草稿，结构检查 pending | 当前输入可读；待确认值没有被自动确认 |
| Isaac Sim 5.1.0 历史兼容回归 | 合成双关节模型完成导入、静态检查、1 秒运行，61 样本 | 仅证明当时的 5.1 兼容分支可运行；不代表 6.1.0 |
| Isaac Sim 6.1.0 官方 API 核对 | 完成文档核对 | 使用版本对应的 URDFImporter、Articulation、SimulationManager 与版本查询接口 |
| Isaac Sim 6.1.0 实际 SDK 运行 | 本机 6.1.0-rc.26：合成模型导入、静态一致性和 1 秒/61 样本运动通过 | 对该构建、固定基座双关节模型的程序链路通过；不等于其他构建、机构或真机已验收 |
| 官方 SysID 扩展 | 6.1.0 自带 2.2.3；官方文档/脚本核对及 --help 通过 | 官方调用接口存在；尚未用真实遥测完成辨识 |
| 实际旧 SDK 的版本拦截 | 本次通过：5.1.0 启动后产生 blocked JSON | 不会把 5.1.0 执行结果写成 6.1.0 通过 |
| wheel 安装验证 | 构建、隔离目录安装、来源注册表读取及 preflight 通过 | 安装包包含程序及对应来源信息 |
| OpenArm 名义模型 | 静态检查与 33 组姿态、693 次连杆 FK 比较通过 | 当前模型导入和独立 FK 代码与 PhysX tensor 一致 |
| OpenArm 真机只读反馈 | 两路恢复 ERROR-ACTIVE；16/16 电机应答、3 轮共 48 条反馈 | 传输和官方 codec 链路可用；坐标、零位及测力精度尚未独立标定 |
| OpenArm 臂反馈输入仿真 | 3 组输入、63 次连杆比较通过；夹爪为仿真参考值 | 真实臂电机数值可进入仿真检查；不是真实夹爪、几何或动力学验收 |
| 实机运动、限位及动力学实验 | 未执行 | 不作物理精度、力矩、接触或控制响应验收声明 |

基础复现：

```bash
python -m pip install -e '.[test,usd,plot]'
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
python examples/run_offline_demo.py --destination /tmp/robotcal-demo-new
```

6.1.0 SDK 复现（独立安装版或 pip 虚拟环境的 Python 均可）：

```bash
python examples/run_isaac_smoke.py --launcher /实际/IsaacSim-6.1.0/python.sh \
  --destination /tmp/robotcal-sdk61-new
```

每次使用新的 destination，避免覆盖实验。SDK 示例只使用带已知定义的合成双关节模型，不连接硬件。报告记录运行版本、步长、重力及关节名称；时间戳、运动方向或重力检查失败时中止。旧 SDK 兼容测试必须显式增加 `--expected-version 5.1.0`。

`tests/fixtures/isaac51_synthetic_reference.usda` 与 `isaac61_synthetic_reference.usda` 是合成模型的 5.1/6.1 导入结果，用于静态一致性和故意损坏的回归测试，不含项目原始 CAD。静态夹具本身不证明运行时物理响应通过；上表中的 6.1 SDK 运行另外进行了实际步进。CI 配置涵盖 Python 3.10、3.12、3.13；其云端执行结果需到 GitHub Actions 查看，本地记录不代替云端结果。
