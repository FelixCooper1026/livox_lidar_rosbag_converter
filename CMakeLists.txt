cmake_minimum_required(VERSION 3.0.2)
project(livox_lidar_rosbag_converter)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找必要的包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  livox_ros_driver2
  pcl_ros
)

# 声明catkin包
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs livox_ros_driver2 pcl_ros
)

# 包含头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(lidar_converter 
  src/lidar_converter.cpp
)

add_executable(bag_converter
  src/bag_converter.cpp
)

# 链接库
target_link_libraries(lidar_converter
  ${catkin_LIBRARIES}
)

target_link_libraries(bag_converter
  ${catkin_LIBRARIES}
)
