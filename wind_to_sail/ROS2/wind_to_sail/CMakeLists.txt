cmake_minimum_required(VERSION 3.5)
project(wind_to_sail)

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

add_executable(pub src/publisher.cpp)
ament_target_dependencies(pub rclcpp std_msgs)
add_executable(sub src/subscriber.cpp)
ament_target_dependencies(sub rclcpp std_msgs)
add_executable(sub_pub src/sub_pub.cpp)
ament_target_dependencies(sub_pub rclcpp std_msgs)

install(TARGETS
  sub_pub
  pub
  sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
