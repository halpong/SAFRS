cmake_minimum_required(VERSION 3.8)
project(main_pi)

# ------------------------------------------------------------------------------
# 1. Build Settings
# ------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# add_compile_options(-Wall -Wextra -Wpedantic)

# ------------------------------------------------------------------------------
# 2. Find Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ------------------------------------------------------------------------------
# 3. Install Python Nodes
# ------------------------------------------------------------------------------
file(GLOB PY_MAIN scripts/*.py)
file(GLOB PY_UTILS scripts/utils/*.py)
file(GLOB PY_ZMQ scripts/zmq/*.py)

install(
  PROGRAMS
    ${PY_MAIN}
    ${PY_UTILS}
    ${PY_ZMQ}
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# 4. Build C++ Nodes (optional)
# ------------------------------------------------------------------------------
set(CPP_NODE src/robot_controller.cpp)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/robot_controller.cpp")
  add_executable(robot_controller ${CPP_NODE})
  ament_target_dependencies(robot_controller
    rclcpp
    std_msgs
    sensor_msgs
  )
  install(
    TARGETS robot_controller
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# ------------------------------------------------------------------------------
# 5. Install Launch / Config / URDF
# ------------------------------------------------------------------------------
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# ------------------------------------------------------------------------------
# 6. Package Export
# ------------------------------------------------------------------------------
ament_package()
