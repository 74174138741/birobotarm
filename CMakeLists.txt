cmake_minimum_required(VERSION 3.20)
project(mujoco_cpp_starter LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ========================
# MuJoCo 路径
# ========================
set(MUJOCO_ROOT "$ENV{MUJOCO_DIR}" CACHE PATH "Path to MuJoCo 3.x")

if(NOT MUJOCO_ROOT)
  set(MUJOCO_ROOT "$ENV{HOME}/mjoco")
endif()

set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT}/include")
set(MUJOCO_LIBRARY "${MUJOCO_ROOT}/lib/libmujoco.so")

if(NOT EXISTS "${MUJOCO_LIBRARY}")
  message(FATAL_ERROR "MuJoCo not found at ${MUJOCO_ROOT}")
endif()

# ========================
# 依赖
# ========================
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
if(TARGET glfw3::glfw)
  set(_MJ_GLFW glfw3::glfw)
elseif(TARGET glfw)
  set(_MJ_GLFW glfw)
else()
  find_library(_MJ_GLFW NAMES glfw glfw3 REQUIRED)
endif()

function(mujoco_target_link target)
  target_include_directories(${target} PRIVATE ${MUJOCO_INCLUDE_DIR})
  target_link_libraries(${target}
    PRIVATE ${MUJOCO_LIBRARY} OpenGL::GL ${_MJ_GLFW})
endfunction()

# 拷贝到可执行文件旁：scenes/ 内含 models/（与仓库 scenes/ 目录布局一致）
function(mujoco_runtime_assets_postbuild target)
  add_custom_command(TARGET ${target} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${MUJOCO_LIBRARY}"
      "$<TARGET_FILE_DIR:${target}>/libmujoco.so"

    COMMAND ${CMAKE_COMMAND} -E rm -rf "$<TARGET_FILE_DIR:${target}>/scenes"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
      "${CMAKE_CURRENT_SOURCE_DIR}/scenes"
      "$<TARGET_FILE_DIR:${target}>/scenes"
  )
endfunction()

# ========================
# 可执行文件
# ========================
add_executable(mj_demo src/main.cpp src/mj_view.cpp)
mujoco_target_link(mj_demo)
mujoco_runtime_assets_postbuild(mj_demo)
