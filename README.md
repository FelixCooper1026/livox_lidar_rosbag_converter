# Livox LiDAR ROS Bag Converter

这个包提供了两种方式将 Livox LiDAR 数据从 `sensor_msgs/PointCloud2` 格式转换为 `livox_ros_driver2/CustomMsg` 格式。

## 功能特点

- 支持实时数据转换（ROS节点方式）
- 支持直接转换 rosbag 文件
- 保持原始话题名和时间戳
- 保留 bag 文件中的其他话题数据

## 依赖项

- ROS Noetic/Melodic
- roscpp
- sensor_msgs
- livox_ros_driver2
- pcl_ros
- rosbag

## 安装

1. 克隆仓库到工作空间：
```bash
cd ~/catkin_ws/src
git clone https://github.com/FelixCooper1026/livox_lidar_rosbag_converter.git
```

2. 编译：
```bash
cd ~/catkin_ws
catkin_make
```

## 使用方法

### 1. 实时转换（ROS节点方式）

```bash
rosrun livox_lidar_rosbag_converter lidar_converter
```

实时转换时：
- 订阅 `/livox/lidar` 话题（PointCloud2 格式）
- 发布 `/livox/lidar_custommsg` 话题（CustomMsg 格式）

### 2. 直接转换 rosbag 文件

```bash
rosrun livox_lidar_rosbag_converter bag_converter <输入bag文件路径> <输出bag文件路径>
```

例如：
```bash
rosrun livox_lidar_rosbag_converter bag_converter input.bag output.bag
```

bag 转换时：
1. 读取 input.bag 文件中的 `/livox/lidar` 话题，将其从 PointCloud2 格式转换为 CustomMsg 格式，并保持话题名不变
2. 保留原始 bag 文件中的所有其他话题（如 `/livox/imu` 等）不变
3. 保持所有消息的原始时间戳

## 许可证

本项目采用 BSD 许可证。详见 [LICENSE](LICENSE) 文件。

## 作者

- FelixCooper1026

## 贡献

欢迎提交 Issue 和 Pull Request！
