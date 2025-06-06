# Livox LiDAR ROS Bag Converter

提供了两种方式将 Livox LiDAR Rosbag 数据在 `sensor_msgs/PointCloud2` 格式和 `livox_ros_driver2/CustomMsg` 格式之间进行转换，目前仅支持通过 livox ros driver2（支持 Livox Mid-360, Livox HAP 激光雷达）采集的数据

## 功能特点

- 支持实时数据转换和发布（ROS节点方式）
- 支持直接转换 rosbag 文件和保存
- 支持双向转换：PointCloud2 ↔ CustomMsg

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

3. 设置环境变量：
```bash
source devel/setup.bash
```

## 使用方法

### 1. PointCloud2 转 CustomMsg

#### 实时转换（ROS节点方式）
```bash
rosrun livox_lidar_rosbag_converter pointcloud2_to_custommsg
```

#### 直接转换 rosbag 文件
```bash
rosrun livox_lidar_rosbag_converter pointcloud2_to_custommsg_bag <输入bag文件路径> <输出bag文件路径>
```

### 2. CustomMsg 转 PointCloud2

#### 实时转换（ROS节点方式）
```bash
rosrun livox_lidar_rosbag_converter custommsg_to_pointcloud2
```

#### 直接转换 rosbag 文件
```bash
rosrun livox_lidar_rosbag_converter custommsg_to_pointcloud2_bag <输入bag文件路径> <输出bag文件路径>
```

## 话题说明

### 实时转换

#### PointCloud2 转 CustomMsg
- 订阅 `/livox/lidar` 话题（PointCloud2 格式）
- 发布 `/livox/lidar_custommsg` 话题（CustomMsg 格式）

#### CustomMsg 转 PointCloud2
- 订阅 `/livox/lidar` 话题（CustomMsg 格式）
- 发布 `/livox/lidar_pointcloud2` 话题（PointCloud2 格式）

### 直接转换rosbag文件

#### PointCloud2 转 CustomMsg
- 读取输入bag文件中的 `/livox/lidar` 话题（PointCloud2 格式）
- 转换为 CustomMsg 格式后写入输出bag文件，保持话题名 `/livox/lidar` 不变
- 保留原始bag文件中的所有其他话题数据
- 保持所有消息的原始时间戳

#### CustomMsg 转 PointCloud2
- 读取输入bag文件中的 `/livox/lidar` 话题（CustomMsg 格式）
- 转换为 PointCloud2 格式后写入输出bag文件，保持话题名 `/livox/lidar` 不变
- 保留原始bag文件中的所有其他话题数据
- 保持所有消息的原始时间戳
