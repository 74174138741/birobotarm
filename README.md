# grinding

MuJoCo 仿真、飞西（Feixi）机械臂场景，以及基于 **ros2_control** 的 MuJoCo 硬件插件与 launch。顶层另有独立 CMake 示例程序 **`mj_demo`**。

## 依赖

| 组件 | 用途 |
|------|------|
| [MuJoCo 3.x](https://mujoco.org/) | 物理仿真；默认路径 `~/mujoco` 或环境变量 `MUJOCO_DIR` |
| CMake ≥ 3.20、C++17 | 顶层 `mj_demo` |
| ROS 2 **Jazzy**、`colcon` | 包 `feixi_ros2_control` |
| OpenGL、GLFW | MuJoCo 可视化 |

## 仓库布局（概要）

```text
.
├── CMakeLists.txt          # 顶层 mujoco_cpp_starter / mj_demo
├── launch.md               # ROS 2：编译、launch 参数、自检（详细）
├── scenes/                 # MJCF / 网格等（机械臂场景）
├── ros2/feixi_ros2_control/# ROS 2 包源码（硬件插件、launch、URDF、脚本）
└── ros2_build_ws/          # colcon 工作区（src 内软链指向 ros2/feixi_ros2_control）
```

## 顶层 C++ 示例（mj_demo）

```bash
export MUJOCO_DIR="${MUJOCO_DIR:-$HOME/mujoco}"
cmake -S . -B build && cmake --build build -j
./build/mj_demo
```

## ROS 2：`feixi_ros2_control`

完整步骤（根目录编译、`ros2 launch`、参数表、Mock、自检）见 **[launch.md](launch.md)**。

最小流程：

```bash
source /opt/ros/jazzy/setup.bash
export MUJOCO_DIR="${MUJOCO_DIR:-$HOME/mujoco}"

colcon --log-base ros2_build_ws/log build \
  --base-paths ros2_build_ws/src \
  --build-base ros2_build_ws/build \
  --install-base ros2_build_ws/install \
  --packages-select feixi_ros2_control

source ./ros2_build_ws/install/setup.bash
ros2 launch feixi_ros2_control feixi_mujoco_ros2_control.launch.py
```

## 许可证

以各子目录 `package.xml` / 源码头注释为准（`feixi_ros2_control` 声明为 Apache-2.0）。
