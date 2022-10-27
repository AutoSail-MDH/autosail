cmake_minimum_required(VERSION 3.8)
project(true_wind_calculation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autosail_message REQUIRED)

add_executable(true_wind src/true_wind_calculation.cpp)
ament_target_dependencies(true_wind rclcpp autosail_message)

ament_package()
