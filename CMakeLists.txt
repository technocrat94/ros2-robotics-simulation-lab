cmake_minimum_required(VERSION 3.8)

project(ur5_moveit_demo)

# ======================================================
# Compiler warnings
# ======================================================

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

# ======================================================
# 找到需要的 ROS / C++ 套件
# ======================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Threads REQUIRED)

# ======================================================
# 建立 move_xyz executable
#
# 原始碼：
# src/move_xyz.cpp
#
# 編譯後程式：
# move_xyz
# ======================================================

add_executable(
  move_xyz
  src/move_xyz.cpp
)

add_executable(
  pick_at_position
  src/pick_at_position.cpp
)

# ======================================================
# ROS dependencies
# ======================================================

ament_target_dependencies(
  move_xyz
  rclcpp
  rclcpp_action
  control_msgs
  moveit_ros_planning_interface
  geometry_msgs
)

ament_target_dependencies(
  pick_at_position
  rclcpp
  rclcpp_action
  control_msgs
  moveit_ros_planning_interface
  geometry_msgs
)

# std::thread 使用的 thread library
target_link_libraries(
  move_xyz
  Threads::Threads
)

target_link_libraries(
  pick_at_position
  Threads::Threads
)

# ======================================================
# 安裝 executable
#
# 這樣 ros2 run 才找得到 move_xyz
# ======================================================

install(
  TARGETS
    move_xyz
    pick_at_position
  DESTINATION
    lib/${PROJECT_NAME}
)

# ======================================================
# 安裝 launch 資料夾
#
# 這樣 ros2 launch 才找得到
# move_xyz.launch.py
# ======================================================

install(
  DIRECTORY
    launch
    urdf
    config
    srdf
    rviz
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
