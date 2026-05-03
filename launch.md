# 使用说明

## 环境

```bash
export MUJOCO_DIR=~/mujoco   # 按实际安装路径修改
export LD_LIBRARY_PATH="$MUJOCO_DIR/lib:${LD_LIBRARY_PATH:-}"
```

## 依赖

```bash
sudo apt install -y libglfw3-dev
```

## 编译

```bash
cmake -S . -B build -DMUJOCO_ROOT="${MUJOCO_DIR:-$HOME/mujoco}"
cmake --build build -j"$(nproc)"
```

## 运行

从项目根目录执行（可执行文件旁已复制 `feixi/` 模型与 `collision` 占位网格）：

```bash
./build/mj_demo1
```

若存在 `src/main.cpp`，会同时生成 `./build/mj_demo`。

## 官方仿真器（可选）

```bash
"$MUJOCO_DIR/bin/simulate" models/feixi/feixi_model.xml
```
