cmake_minimum_required(VERSION 3.8)
project(arm_execution_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

install(DIRECTORY include/
  DESTINATION include
)

add_executable(arm_execution_manager
  src/arm_execution_manager.cpp
)

ament_target_dependencies(arm_execution_manager
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  moveit_core
  tf2
)

target_include_directories(arm_execution_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS 
  arm_execution_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
