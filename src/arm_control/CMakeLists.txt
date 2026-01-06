cmake_minimum_required(VERSION 3.8)
project(arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(PiPCA9685 REQUIRED)

add_library(hardware SHARED src/hardware.cpp)
target_include_directories(hardware PUBLIC include)
ament_target_dependencies(hardware hardware_interface pluginlib rclcpp rclcpp_lifecycle PiPCA9685)
target_link_libraries(hardware
  PiPCA9685::PiPCA9685
)

pluginlib_export_plugin_description_file(hardware_interface hardware.xml)

install(TARGETS hardware
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
