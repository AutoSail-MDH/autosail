cmake_minimum_required(VERSION 3.5)
project(test_sub)

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
find_package(message_filters REQUIRED)

add_executable(approx src/approx_2sub_pub.cpp)
ament_target_dependencies(approx rclcpp std_msgs message_filters)
add_executable(exact src/exact_2sub_pub.cpp)
ament_target_dependencies(exact rclcpp std_msgs message_filters)
install(TARGETS
  approx
  exact
  DESTINATION lib/${PROJECT_NAME})

ament_package()
