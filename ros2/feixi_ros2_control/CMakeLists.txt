cmake_minimum_required(VERSION 3.16)
project(feixi_ros2_control)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenGL REQUIRED)

find_package(glfw3 QUIET CONFIG)
if(TARGET glfw3::glfw)
  set(FEIXI_GLFW_TARGET glfw3::glfw)
elseif(TARGET glfw)
  set(FEIXI_GLFW_TARGET glfw)
else()
  find_library(FEIXI_GLFW_LIBRARY NAMES glfw glfw3 REQUIRED)
  add_library(feixi_glfw_import UNKNOWN IMPORTED)
  set_target_properties(feixi_glfw_import PROPERTIES IMPORTED_LOCATION "${FEIXI_GLFW_LIBRARY}")
  set(FEIXI_GLFW_TARGET feixi_glfw_import)
endif()

if(DEFINED ENV{MUJOCO_DIR} AND NOT "$ENV{MUJOCO_DIR}" STREQUAL "")
  set(MUJOCO_ROOT "$ENV{MUJOCO_DIR}")
else()
  set(MUJOCO_ROOT "$ENV{HOME}/mujoco")
endif()

set(MUJOCO_INCLUDE "${MUJOCO_ROOT}/include")
set(MUJOCO_LIB_PATH "${MUJOCO_ROOT}/lib/libmujoco.so")
if(NOT EXISTS "${MUJOCO_LIB_PATH}")
  message(FATAL_ERROR
    "MuJoCo missing at ${MUJOCO_LIB_PATH}. Set MUJOCO_DIR before building (CMake configure pass).")
endif()

add_library(feixi_mujoco_hardware_plugin SHARED
  src/feixi_mujoco_hardware.cpp
  src/mj_view.cpp)
target_compile_features(feixi_mujoco_hardware_plugin PRIVATE cxx_std_17)
target_include_directories(feixi_mujoco_hardware_plugin PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_include_directories(feixi_mujoco_hardware_plugin PRIVATE ${MUJOCO_INCLUDE})
target_link_libraries(feixi_mujoco_hardware_plugin PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  "${MUJOCO_LIB_PATH}")
target_link_libraries(feixi_mujoco_hardware_plugin PRIVATE OpenGL::GL "${FEIXI_GLFW_TARGET}")

ament_target_dependencies(feixi_mujoco_hardware_plugin PUBLIC
  hardware_interface
  pluginlib
  rclcpp)

pluginlib_export_plugin_description_file(hardware_interface feixi_mujoco_hardware.xml)

install(
  DIRECTORY include/
  DESTINATION include)
install(TARGETS feixi_mujoco_hardware_plugin
  DESTINATION lib)
install(
  DIRECTORY urdf launch config
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/send_feixi_demo_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME send_feixi_demo_trajectory)
install(PROGRAMS scripts/stream_feixi_joint_commands.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME stream_feixi_joint_commands)

set(_GRIND_SCENES "${CMAKE_CURRENT_SOURCE_DIR}/../../scenes")
if(EXISTS "${_GRIND_SCENES}/arm_with_gripper.xml")
  install(DIRECTORY "${_GRIND_SCENES}/" DESTINATION share/${PROJECT_NAME}/scenes)
endif()

ament_export_include_directories(include)
ament_export_dependencies(hardware_interface pluginlib rclcpp)
ament_export_libraries(feixi_mujoco_hardware_plugin)

ament_package()
