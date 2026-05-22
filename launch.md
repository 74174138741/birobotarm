# 编译与运行

**环境**：`~/.bashrc` 中已配置 `MUJOCO_DIR`、`LD_LIBRARY_PATH` 与 `source /opt/ros/jazzy/setup.bash`，新开终端即可用。若未配置，请先写入后再继续。

配置方法为
```bash
MUJOCO_DIR="${MUJOCO_DIR:-$HOME/mujoco}"
export LD_LIBRARY_PATH="${MUJOCO_DIR}/lib:${LD_LIBRARY_PATH:-}"
source /opt/ros/jazzy/setup.bash
```

---

## 非 ROS（MuJoCo 简单测试）

```bash
cmake -S test -B test/build && cmake --build test/build -j
./test/build/mj_demo
```

改动了源码或场景后需重新构建；未改动时直接运行 `./test/build/mj_demo`。

---

## ROS 2

### 编译

在仓库根目录 `grinding/`：

```bash
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PWD/log/launch}"   # 可选，launch 日志目录

colcon build --packages-select feixi_mujoco_control feixi_trajectory
source install/setup.bash
```

### 运行仿真

```bash
source install/setup.bash
ros2 launch feixi_mujoco_control feixi_mujoco_ros2_control.launch.py
```

查看全部参数：`--show-args`

### Launch 参数与默认值

1. **`mjcf_model_path`**（默认 `scenes/arm_with_gripper.xml`）  
   MuJoCo 场景；相对路径解析到 `share/feixi_mujoco_control/` 下。

2. **`control_mode`**（默认 `effort`）  
   `position` | `effort`。力矩模式下 PD 由 JTC gains 承担，适合接动力学。

3. **`joint_commands`**（默认 `trajectory`）  
   `trajectory`：JointTrajectoryController；`stream`：ForwardCommandController（需发 `Float64MultiArray`，且须 `control_mode:=position`）。

4. **`init_pose_q`**（默认 `0,0,0,0,0,0,0,0.05`）  
   启动时写入 MuJoCo 的 8 个关节角（rad）：joint1..7 + 夹爪。

5. **`init_lock_writes`**（默认 `500`）  
   启动后锁定初始位姿的 write 次数（500 Hz 下约 1 s）；`0` 则只 seed 一次。

6. **`mujoco_joint_actuation`**（默认 `pd_torque`）  
   仅 `control_mode:=position` 时有效：`pd_torque` 插件内 PD；`direct` 运动学对齐关节角。`effort` 模式下忽略。

7. **`enable_mujoco_viewer`**（默认 `true`）  
   `true` | `false`，是否打开 MuJoCo 原生可视化窗口。

8. **`viewer_width` / `viewer_height`**（默认 `960` / `720`）  
   可视化窗口分辨率（像素）。

9. **`run_test`**（默认 `false`）  
   `true` 时 launch 后自动发 demo 轨迹或 stream 指令做自检。

10. **`reference_trajectory_test`**（默认 `false`）  
    `true` 时 launch 后自动发 3 路点参考轨迹（配合 `dynamics_mode:=trajectory`）。

不带任何参数启动时，等价于：

```bash
ros2 launch feixi_mujoco_control feixi_mujoco_ros2_control.launch.py \
  mjcf_model_path:=scenes/arm_with_gripper.xml \
  control_mode:=effort \
  joint_commands:=trajectory \
  init_pose_q:="0,0,0,0,0,0,0,0.05" \
  init_lock_writes:=500 \
  mujoco_joint_actuation:=pd_torque \
  enable_mujoco_viewer:=true \
  viewer_width:=960 \
  viewer_height:=720
```

（`control_mode:=effort` 时 `mujoco_joint_actuation` 被忽略。）

示例：打开可视化 + 力矩模式 + 自动 demo 测试

```bash
ros2 launch feixi_mujoco_control feixi_mujoco_ros2_control.launch.py run_test:=true
```

### 另开终端：手动发送指令

```bash
source install/setup.bash
ros2 run feixi_trajectory send_feixi_demo_trajectory
ros2 run feixi_trajectory send_feixi_reference_trajectory
ros2 run feixi_trajectory stream_feixi_joint_commands
```

`stream_feixi_joint_commands` 需在 launch 里使用 `control_mode:=position` 且 `joint_commands:=stream`，默认 topic：`/feixi_forward_position_controller/commands`。

### Mock 栈（无 MuJoCo）

```bash
ros2 launch feixi_mujoco_control feixi_mock_ros2_control.launch.py
ros2 run feixi_trajectory send_feixi_demo_trajectory -- --seven-dof
```
