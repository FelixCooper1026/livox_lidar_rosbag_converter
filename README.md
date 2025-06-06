# Livox LiDAR ROS Bag Converter

这个包提供了两种方式将 Livox LiDAR 数据从 `sensor_msgs/PointCloud2` 格式转换为 `livox_ros_driver2/CustomMsg` 格式：

## 1. 实时转换（ROS节点方式）

```bash
rosrun livox_lidar_rosbag_converter lidar_converter
```

实时转换时：
- 订阅 `/livox/lidar` 话题（PointCloud2 格式）
- 发布 `/livox/lidar_custommsg` 话题（CustomMsg 格式）

## 2. 直接转换 rosbag 文件

```bash
rosrun livox_lidar_rosbag_converter bag_converter <输入bag文件路径> <输出bag文件路径>
```

例如：
```bash
rosrun livox_lidar_rosbag_converter bag_converter input.bag output.bag
```

直接转换时：
1. 读取 input.bag 文件中的 `/livox/lidar` 话题，将其从 PointCloud2 格式转换为 CustomMsg 格式，并保持话题名 `/livox/lidar` 不变
2. 保留原始 bag 文件中的所有其他话题（如 `/livox/imu` 等）
3. 保持所有消息的原始时间戳
