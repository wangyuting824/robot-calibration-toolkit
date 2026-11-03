# 依赖与第三方资料

本仓库的程序为项目代码，未捆绑机器人原始 STEP、原始文档、真实日志或 NVIDIA SDK。

私有现场硬盘包另外包含用户提供的旭创资料，公开仓库不包含这些资料。Linux 运行环境来自 [Astral python-build-standalone](https://docs.astral.sh/python-build-standalone/)，CPython 3.12.16 / 20261026；官方归档 SHA-256、来源和依赖版本记录在 runtime/RUNTIME_SOURCE.json、requirements-lock.txt，完整 wheelhouse 随包提供。Python 和依赖许可保留在 runtime 内的 LICENSE 与各 dist-info/licenses 中。Isaac、显卡驱动、ROS、厂商 SDK 独立安装。

| 依赖 | 作用 | 上游 |
|---|---|---|
| NumPy | 数组和线性代数 | https://numpy.org/ |
| SciPy | 优化、旋转与线性代数 | https://scipy.org/ |
| PyYAML | YAML 配置读写 | https://pyyaml.org/ |
| openpyxl | XLSX 参数主表 | https://openpyxl.readthedocs.io/ |
| pytest（开发） | 离线测试 | https://docs.pytest.org/ |
| NVIDIA Isaac Sim（外部安装） | 导入、物理运行及官方辨识工具 | https://docs.isaacsim.omniverse.nvidia.com/ |
| MuJoCo（可选） | URDF 编译、独立 FK 比较、仿真步进与查看器 | https://mujoco.readthedocs.io/ |
| trimesh、pycollada（可选） | 把不受 MuJoCo 直接支持的 DAE 几何转换为 OBJ | https://trimesh.org/ 、https://pycollada.readthedocs.io/ |
| Pillow（可选） | 保存模型渲染截图 | https://pillow.readthedocs.io/ |
| ROS 2 rclpy、sensor_msgs（外部安装） | 订阅已有 JointState 状态流 | https://github.com/ros2/rclpy 、https://github.com/ros2/common_interfaces |

依赖的许可证以各安装发行包为准。第三方文档通过链接引用，未将其全文复制进仓库。原始机器人资料的权利及使用范围独立于工具包源码。
