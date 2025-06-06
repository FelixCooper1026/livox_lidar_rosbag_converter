# Livox LiDAR ROS Bag Converter

提供了两种方式将 Livox LiDAR Rosbag 数据从 `sensor_msgs/PointCloud2` 格式转换为 `livox_ros_driver2/CustomMsg` 格式。
目前仅支持通过 livox ros driver2（支持 Livox Mid-360, Livox HAP 激光雷达)采集的数据

## 功能特点

- 支持实时数据转换和发布（ROS节点方式）
- 支持直接转换 rosbag 文件和保存

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

直接转换时：
1. 读取 input.bag 文件中的 `/livox/lidar` 话题，将其从 PointCloud2 格式转换为 CustomMsg 格式，并保持话题名`/livox/lidar`不变
2. 保留原始 bag 文件中的所有其他话题（如 `/livox/imu` 等）不变
3. 保持所有消息的原始时间戳
