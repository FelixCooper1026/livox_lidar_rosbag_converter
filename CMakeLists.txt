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
add_executable(pointcloud2_to_custommsg 
  src/pointcloud2_to_custommsg.cpp
)

add_executable(pointcloud2_to_custommsg_bag
  src/pointcloud2_to_custommsg_bag.cpp
)

add_executable(custommsg_to_pointcloud2 
  src/custommsg_to_pointcloud2.cpp
)

add_executable(custommsg_to_pointcloud2_bag
  src/custommsg_to_pointcloud2_bag.cpp
)

# 链接库
target_link_libraries(pointcloud2_to_custommsg
  ${catkin_LIBRARIES}
)

target_link_libraries(pointcloud2_to_custommsg_bag
  ${catkin_LIBRARIES}
)

target_link_libraries(custommsg_to_pointcloud2
  ${catkin_LIBRARIES}
)

target_link_libraries(custommsg_to_pointcloud2_bag
  ${catkin_LIBRARIES}
)
