cmake_minimum_required(VERSION 3.8)
project(pulsar_cpp_code)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(tick_to_odom src/tick_to_odom.cpp)

ament_target_dependencies(tick_to_odom
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

install(
  TARGETS tick_to_odom
  
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
