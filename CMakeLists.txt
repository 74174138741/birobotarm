cmake_minimum_required(VERSION 3.20)
project(mujoco_cpp_starter LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ========================
# MuJoCo 路径
# ========================
set(MUJOCO_ROOT "$ENV{MUJOCO_DIR}" CACHE PATH "Path to MuJoCo 3.x")

if(NOT MUJOCO_ROOT)
  set(MUJOCO_ROOT "$ENV{HOME}/mujoco")
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

# ========================
# 公共函数（关键优化）
# ========================
function(add_mujoco_exe target src)
  add_executable(${target} ${src})

  target_include_directories(${target} PRIVATE ${MUJOCO_INCLUDE_DIR})

  target_link_libraries(${target}
    PRIVATE
    ${MUJOCO_LIBRARY}
    OpenGL::GL
    ${_MJ_GLFW}
  )

  # 拷贝资源
  add_custom_command(TARGET ${target} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${MUJOCO_LIBRARY}"
      "$<TARGET_FILE_DIR:${target}>/libmujoco.so"

    COMMAND ${CMAKE_COMMAND} -E make_directory
      "$<TARGET_FILE_DIR:${target}>/feixi"

    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${CMAKE_CURRENT_SOURCE_DIR}/models/feixi/feixi_model.xml"
      "$<TARGET_FILE_DIR:${target}>/feixi/feixi_model.xml"

    COMMAND ${CMAKE_COMMAND} -E copy_directory
      "${CMAKE_CURRENT_SOURCE_DIR}/models/feixi/visual"
      "$<TARGET_FILE_DIR:${target}>/feixi/visual"

    # MJCF references collision/*.obj but repo only ships visual/; reuse meshes for load + run.
    COMMAND ${CMAKE_COMMAND} -E make_directory
      "$<TARGET_FILE_DIR:${target}>/feixi/collision"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
      "${CMAKE_CURRENT_SOURCE_DIR}/models/feixi/visual"
      "$<TARGET_FILE_DIR:${target}>/feixi/collision"
  )
endfunction()

# ========================
# 可执行文件
# ========================
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")
  add_mujoco_exe(mj_demo src/main.cpp)
endif()
add_mujoco_exe(mj_demo1 src/main1.cpp)