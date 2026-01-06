cmake_minimum_required(VERSION 3.8)
project(arm_mtc_tasks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Manipulation.action"
  DEPENDENCIES
)

# ------------------------------------
# Task library
# ------------------------------------
add_library(mtc_tasks
  src/tasks/pickup_task.cpp
)

target_include_directories(mtc_tasks PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mtc_tasks
  rclcpp
  moveit_task_constructor_core
)

# Make sure tasks can see generated action headers
rosidl_target_interfaces(mtc_tasks
  ${PROJECT_NAME} rosidl_typesupport_cpp
)

# ------------------------------------
# Task server executable
# ------------------------------------
add_executable(task_server
  src/server/task_server.cpp
)

target_include_directories(task_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(task_server
  rclcpp
  rclcpp_action
  moveit_task_constructor_core
)

# Link task library
target_link_libraries(task_server
  mtc_tasks
)

# Action server also needs action types
rosidl_target_interfaces(task_server
  ${PROJECT_NAME} rosidl_typesupport_cpp
)

# ------------------------------------
# Install targets
# ------------------------------------
install(
  TARGETS
    mtc_tasks
    task_server
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# ------------------------------------
# Export for downstream packages
# ------------------------------------
ament_export_include_directories(include)
ament_export_libraries(mtc_tasks)
ament_export_dependencies(
  rclcpp
  moveit_task_constructor_core
  rosidl_default_runtime
)

ament_package()
