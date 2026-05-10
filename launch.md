# 编译与运行

## 非 ROS（顶层 CMake）

```bash
#这里可以对单mujoco环境进行测试
cmake -S . -B build && cmake --build build -j
./build/mj_demo
```

---



### 编译

**在仓库根目录 `grinding/`（不必 `cd ros2_build_ws`）：**

```bash
source /opt/ros/jazzy/setup.bash
export MUJOCO_DIR="${MUJOCO_DIR:-$HOME/mujoco}"

colcon --log-base ros2_build_ws/log build \
  --base-paths ros2_build_ws/src \
  --build-base ros2_build_ws/build \
  --install-base ros2_build_ws/install \
  --packages-select feixi_ros2_control

source ./ros2_build_ws/install/setup.bash
```

`--packages-select` 只能写**包名** `feixi_ros2_control`，不能写路径。


### 查看 launch 全部参数

```bash

source ./ros2_build_ws/install/setup.bash

ros2 launch feixi_ros2_control feixi_mujoco_ros2_control.launch.py --show-args
```


 `mjcf_model_path` 这里可以切选用的mujoco场景  `scenes/arm_with_gripper.xml`

 `control_mode`
      `position` 
      `effort`：

 `joint_commands` 
    `trajectory`：JointTrajectoryController；
    `stream`：ForwardCommandController（需发 `Float64MultiArray`） 


`init_pose_q` 
    `0,-0.5,0,1.2,0,0.8,0,0.05`

`init_lock_writes` 
     `500` | 启动阶段若干 write 内先做初始位姿锁定（0 则只做一次 seed）


`mujoco_joint_actuation` 
     `pd_torque` 电机 PD；
     `direct` 每拍运动学对齐关节角（几乎不抖，臂端接触/力不真实） 

`enable_mujoco_viewer` 
    `false` 
    `true` 打开 MuJoCo GLFW 窗口 
`viewer_width` 
`viewer_height` 
    `960` / `720`
`run_test` 
    `false` 
    `true`



```bash
ros2 launch feixi_ros2_control feixi_mujoco_ros2_control.launch.py enable_mujoco_viewer:=true mujoco_joint_actuation:=direct
joint_commands:=stream 
run_test:=true
mujoco_joint_actuation:=direct
control_mode:=effort
```



### 另开终端：手动发送指令

```bash
ros2 run feixi_ros2_control send_feixi_demo_trajectory
ros2 run feixi_ros2_control stream_feixi_joint_commands
```

`stream_feixi_joint_commands` 需在 launch 里使用 `joint_commands:=stream`，默认 topic：`/feixi_forward_position_controller/commands`。


