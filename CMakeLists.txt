cmake_minimum_required(VERSION 3.0.2)
project(livox_lidar_rosbag_converter)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  sensor_msgs
  livox_ros_driver2
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rosbag sensor_msgs livox_ros_driver2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

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

foreach(target
    pointcloud2_to_custommsg
    pointcloud2_to_custommsg_bag
    custommsg_to_pointcloud2
    custommsg_to_pointcloud2_bag)
  target_link_libraries(${target}
    ${catkin_LIBRARIES}
  )
  if(catkin_EXPORTED_TARGETS)
    add_dependencies(${target}
      ${catkin_EXPORTED_TARGETS}
    )
  endif()
endforeach()
