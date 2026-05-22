# grinding

MuJoCo 仿真、（Feixi）机械臂场景，以及基于 **ros2_control** 的 MuJoCo 硬件插件与 launch。仓库根目录为标准 **colcon workspace**；另有独立 CMake 测试程序 **`mj_demo`**。

## 依赖

| 组件 | 用途 |
|------|------|
| [MuJoCo 3.x](https://mujoco.org/) | 物理仿真；默认路径 `~/mujoco` 或环境变量 `MUJOCO_DIR` |
| CMake ≥ 3.20、C++17 | 非 ROS 测试 `mj_demo` |
| ROS 2 **Jazzy**、`colcon` | `feixi_mujoco_control`、`feixi_trajectory` |
| OpenGL、GLFW | MuJoCo 可视化 |
| Pinocchio | `feixi_mujoco_control` 动力学（`pkg-config pinocchio`） |

## 仓库布局

```text
.
├── test/                       # MuJoCo 简单测试（CMake 工程：main + mj_view）
├── scenes/                     # 共享 MJCF / 网格（仓库根，两处共用）
├── src/
│   ├── feixi_mujoco_control/   # ros2_control ↔ MuJoCo 硬件插件、launch、URDF
│   └── feixi_trajectory/       # 轨迹 / 关节指令发送（Python）
├── build/ install/ log/        # colcon 产物（不提交 git）
└── launch.md                   # ROS 2 编译、launch 参数、自检
```

## 非 ROS 测试（mj_demo）

```bash
export MUJOCO_DIR="${MUJOCO_DIR:-$HOME/mujoco}"
cmake -S test -B test/build && cmake --build test/build -j
./test/build/mj_demo
```

## ROS 2

完整步骤见 **[launch.md](launch.md)**。

```bash
source /opt/ros/jazzy/setup.bash
export MUJOCO_DIR="${MUJOCO_DIR:-$HOME/mujoco}"

cd /home/star/program/grinding
colcon build --packages-select feixi_mujoco_control feixi_trajectory
source install/setup.bash

ros2 launch feixi_mujoco_control feixi_mujoco_ros2_control.launch.py
```

另开终端发送轨迹：

```bash
source install/setup.bash
ros2 run feixi_trajectory send_feixi_demo_trajectory
ros2 run feixi_trajectory stream_feixi_joint_commands
```

可选：launch 前设置 launch 日志目录

```bash
export ROS_LOG_DIR="$PWD/log/launch"
```

## 许可证

以各子目录 `package.xml` / 源码头注释为准（Apache-2.0）。
