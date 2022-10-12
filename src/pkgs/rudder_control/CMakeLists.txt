cmake_minimum_required(VERSION 3.5)
project(rudder_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(rudder_control src/rudder_control.cpp)
ament_target_dependencies(rudder_control rclcpp std_msgs)
add_executable(set_next_position src/set_next_position.cpp)
ament_target_dependencies(set_next_position rclcpp std_msgs)
install(TARGETS
rudder_control
  set_next_position
  DESTINATION lib/${PROJECT_NAME})

ament_package()
