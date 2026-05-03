#!/usr/bin/env bash
set -euo pipefail

# ===== 可改参数 =====
MUJOCO_VERSION="3.2.6"
PROJECT_DIR="$HOME/program/real/programming/grinding"
MUJOCO_DIR="$HOME/mujoco"
# ====================

echo "[1/7] 安装基础依赖..."
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git pkg-config curl tar \
  libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev \
  libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev

echo "[2/7] 下载 MuJoCo ${MUJOCO_VERSION}..."
cd "$HOME"
TMP_TAR="mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz"
URL="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/${TMP_TAR}"

curl -fL "$URL" -o "$TMP_TAR"

echo "[3/7] 安装到 ~/mujoco ..."
rm -rf "$MUJOCO_DIR"
mkdir -p "$MUJOCO_DIR"
tar -xzf "$TMP_TAR" -C "$MUJOCO_DIR" --strip-components=1
rm -f "$TMP_TAR"

echo "[4/7] 写入环境变量..."
if ! grep -q 'export MUJOCO_DIR="$HOME/mujoco"' "$HOME/.bashrc"; then
  echo 'export MUJOCO_DIR="$HOME/mujoco"' >> "$HOME/.bashrc"
fi
if ! grep -q 'export LD_LIBRARY_PATH="$MUJOCO_DIR/lib:$LD_LIBRARY_PATH"' "$HOME/.bashrc"; then
  echo 'export LD_LIBRARY_PATH="$MUJOCO_DIR/lib:$LD_LIBRARY_PATH"' >> "$HOME/.bashrc"
fi

# 当前 shell 也生效
export MUJOCO_DIR="$MUJOCO_DIR"
export LD_LIBRARY_PATH="$MUJOCO_DIR/lib:${LD_LIBRARY_PATH:-}"

echo "[5/7] 检查 MuJoCo 文件..."
test -f "$MUJOCO_DIR/include/mujoco/mujoco.h"
test -f "$MUJOCO_DIR/lib/libmujoco.so"
echo "MuJoCo 安装检查通过。"

echo "[6/7] 重新构建项目..."
cd "$PROJECT_DIR"
rm -rf build
cmake -S . -B build -DMUJOCO_ROOT="$MUJOCO_DIR"
cmake --build build -j"$(nproc)"

echo "[7/7] 运行测试..."
./build/mj_demo --headless
./build/mj_demo1 --headless

echo "全部完成。"