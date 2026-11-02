# 目录与现有资料

原始资料与软件工具包并列存放：

```text
项目目录/
├── 机器人 URDF 建模资料确认清单.docx    资料要求与待确认事项
├── robot_urdf_external_joints.zip       对外 URDF、网格与说明
├── 机器人总装（对外）.STEP             CAD 总装
└── robot_calibration_toolkit/
    ├── run.py                         统一命令入口
    ├── pyproject.toml                 安装和依赖
    ├── core/                          格式、坐标、运动学、动力学与来源记录
    ├── adapters/                      回放、Isaac、待接入硬件接口
    ├── programs/
    │   ├── common/                    采集、执行、报告、归档
    │   ├── stage_01_definition/       结构检查与 URDF 生成
    │   ├── stage_02_kinematics/        反馈、限位、位姿标定与验证
    │   └── stage_03_isaac_dynamics/    Isaac、动力学、驱动与接触
    ├── configs/                       机器人和实验配置，未知值留空
    ├── templates/                     空白参数工作簿和数据模板
    ├── inputs/
    │   ├── source_manifest.yaml       原始资料路径
    │   ├── baseline_model/            本地解压模型，不上传
    │   ├── confirmed_parameters/      本地待填/已确认主表，不上传
    │   └── measurements/              本地实测数据，不上传
    ├── examples/                      合成数据与离线/SDK 自检
    ├── tests/                         自动检查与合成静态 USD 夹具
    ├── docs/                          来源、数据、接口、验收和交付说明
    ├── outputs/                       本地运行证据，不上传
    └── deliverables/                  按 D01–D06 打包的实际交付，不上传
```

GitHub 保存软件、空白模板、公开文档链接与合成测试夹具；原始 DOCX、STEP、机器人 ZIP、实测数据由项目本地管理。

原始文档提到的结构主表需要取得或由项目核实填写；从现有 URDF 生成的未确认工作簿不能替代原厂确认资料。各程序与 D01–D06 的对应见 [交付映射](delivery_map.md)。不同轴数和尺寸优先通过配置表达；硬件协议需适配器，闭环、浮动基座等需要相应模型和经过验证的实现。
