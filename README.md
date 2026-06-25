# Livox LiDAR ROS Bag 格式转换工具（ROS 2 版本）

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20|%20Iron%20|%20Rolling-blue)](https://docs.ros.org/en/rolling/index.html)

本包用于在 `sensor_msgs/msg/PointCloud2` 与 `livox_ros_driver2/msg/CustomMsg` 之间相互转换，适配 **ROS 2** 环境下的 Livox LiDAR 数据（如 Mid-360、HAP）。

> 🔔 **ROS 1 用户请使用 `main` 分支**：  
> `git checkout main` 或访问仓库首页查看 ROS 1 说明。

---

## 功能特性

- 🔄 实时转换节点  
- 📦 直接处理 rosbag2 目录并保存  
- ⬅️➡️ 支持双向转换：PointCloud2 ↔ CustomMsg  
- ✅ 转换失败时不生成目标输出目录

---

## 依赖要求

- ROS 2 Humble / Iron / Rolling（或其他版本）  
- `livox_ros_driver2`（ROS 2 版本，需提前安装）  
- `rosbag2` 相关包  

---

## 安装步骤

```bash
# 进入你的 ROS 2 工作空间 src 目录
cd ~/ros2_ws/src

# 克隆本仓库的 ros2 分支
git clone -b ros2 https://github.com/FelixCooper1026/livox_lidar_rosbag_converter.git

# 编译
cd ~/ros2_ws
colcon build --packages-select livox_lidar_rosbag_converter

# 设置环境
source install/setup.bash
```

---

## 使用方法

### 1. 实时转换（ROS 2 节点）

#### PointCloud2 → CustomMsg

```bash
ros2 run livox_lidar_rosbag_converter pointcloud2_to_custommsg
```

#### CustomMsg → PointCloud2

```bash
ros2 run livox_lidar_rosbag_converter custommsg_to_pointcloud2
```

### 2. 直接转换 rosbag2 目录

**重要：** 输出路径必须是一个**不存在的目录**，程序会自动创建。

#### PointCloud2 → CustomMsg

```bash
ros2 run livox_lidar_rosbag_converter pointcloud2_to_custommsg_bag <输入bag目录> <输出bag目录>
```

#### CustomMsg → PointCloud2

```bash
ros2 run livox_lidar_rosbag_converter custommsg_to_pointcloud2_bag <输入bag目录> <输出bag目录>
```

**示例：**

```bash
# 假设有一个 rosbag2 目录 ./my_livox_bag
ros2 run livox_lidar_rosbag_converter pointcloud2_to_custommsg_bag ./my_livox_bag ./my_livox_bag_custom
```

---

## 话题说明

### 实时转换节点

| 转换方向                 | 订阅话题类型与名称                    | 发布话题类型与名称                         |
|--------------------------|---------------------------------------|---------------------------------------------|
| PointCloud2 → CustomMsg  | `sensor_msgs/msg/PointCloud2` 于 `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` 于 `/livox/lidar_custommsg` |
| CustomMsg → PointCloud2  | `livox_ros_driver2/msg/CustomMsg` 于 `/livox/lidar` | `sensor_msgs/msg/PointCloud2` 于 `/livox/lidar_pointcloud2` |

### rosbag2 直接转换

- 程序会自动探测输入 bag 中的第一个有效话题（PointCloud2 或 CustomMsg）。  
- 转换后的消息仍写入原话题名 `/livox/lidar`。  
- 其他话题（如 IMU、诊断信息）及时间戳将被完整保留。
- `PointCloud2 → CustomMsg` 要求输入点云为 `livox_ros_driver2` 的 `xfer_format=0`（`PointXYZRTLT`）字段布局；字段不匹配时会打印实际字段与期望字段差异。
- 离线转换会先写入临时目录，全部成功后才生成目标输出目录；如果输入话题缺失或转换中断，不会留下新的目标输出目录。
- 进度条仅显示转换进度、百分比和消息计数，避免窄终端下自动换行。

---

## 自定义话题名称

如需修改订阅/发布的话题名称，请在源代码中调整对应的常量字符串（位于各 `.cpp` 文件顶部）。

---

## 许可证

本项目使用 [MIT License](LICENSE)。

---

## 贡献与反馈

欢迎提交 Issue 或 Pull Request 帮助改进此工具。  
仓库地址：[https://github.com/FelixCooper1026/livox_lidar_rosbag_converter](https://github.com/FelixCooper1026/livox_lidar_rosbag_converter)
