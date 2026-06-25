cmake_minimum_required(VERSION 3.8)
project(livox_lidar_rosbag_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

include_directories(include)

macro(add_converter_executable target)
  ament_target_dependencies(${target}
    rclcpp
    sensor_msgs
    livox_ros_driver2
  )
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
endmacro()

add_executable(pointcloud2_to_custommsg src/pointcloud2_to_custommsg.cpp)
add_converter_executable(pointcloud2_to_custommsg)

add_executable(custommsg_to_pointcloud2 src/custommsg_to_pointcloud2.cpp)
add_converter_executable(custommsg_to_pointcloud2)

add_executable(pointcloud2_to_custommsg_bag src/pointcloud2_to_custommsg_bag.cpp)
ament_target_dependencies(pointcloud2_to_custommsg_bag
  rclcpp
  sensor_msgs
  livox_ros_driver2
  rosbag2_cpp
  rosbag2_storage
)
target_include_directories(pointcloud2_to_custommsg_bag PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(custommsg_to_pointcloud2_bag src/custommsg_to_pointcloud2_bag.cpp)
ament_target_dependencies(custommsg_to_pointcloud2_bag
  rclcpp
  sensor_msgs
  livox_ros_driver2
  rosbag2_cpp
  rosbag2_storage
)
target_include_directories(custommsg_to_pointcloud2_bag PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  pointcloud2_to_custommsg
  pointcloud2_to_custommsg_bag
  custommsg_to_pointcloud2
  custommsg_to_pointcloud2_bag
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
