cmake_minimum_required(VERSION 3.5)
project(orbslam3)

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
find_package(sensor_msgs REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(realsense2_description REQUIRED)
find_package(realsense2_camera REQUIRED)

add_executable(orbslam3 src/orbslam3.cpp)
ament_target_dependencies(orbslam3 rclcpp std_msgs sensor_msgs realsense2_camera_msgs realsense2_description realsense2_camera)

install(TARGETS
  orbslam3
  DESTINATION lib/${PROJECT_NAME})

ament_package()
