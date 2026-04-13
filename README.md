# Livox LiDAR ROS Bag 格式转换工具

本仓库提供在 `sensor_msgs/PointCloud2` 与 `livox_ros_driver2/CustomMsg` 之间相互转换的工具，支持 **ROS 1** 与 **ROS 2** 两套环境。  
适用于 Livox Mid-360、HAP 等通过 `livox_ros_driver2` 采集的数据。

---

## 💡 提示：ROS 1 与 ROS 2 代码位于不同分支

- **`main` 分支（当前分支）**：ROS 1 版本的 `catkin` 包。  
- **`ros2` 分支**：ROS 2 版本的 `ament_cmake` 包，根目录即为完整 ROS 2 包。

请根据你使用的 ROS 版本克隆或切换对应分支。

---

## 功能特点

- ✅ 实时转换（ROS 节点模式）  
- ✅ 直接转换 rosbag 文件并保存  
- ✅ 双向转换：PointCloud2 ↔ CustomMsg  

---

## 安装与使用（ROS 1）

### 1. 安装依赖

- ROS 1（Noetic / Melodic）  
- `livox_ros_driver2`（ROS 1 版本）  
- `pcl_conversions`

### 2. 编译

```bash
cd ~/catkin_ws/src
git clone -b main https://github.com/FelixCooper1026/livox_lidar_rosbag_converter.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 3. 使用方法

#### 实时转换（节点）

| 转换方向                 | 命令                                               |
|--------------------------|----------------------------------------------------|
| PointCloud2 → CustomMsg  | `rosrun livox_lidar_rosbag_converter pointcloud2_to_custommsg` |
| CustomMsg → PointCloud2  | `rosrun livox_lidar_rosbag_converter custommsg_to_pointcloud2` |

#### 直接转换 rosbag 文件

```bash
# PointCloud2 → CustomMsg
rosrun livox_lidar_rosbag_converter pointcloud2_to_custommsg_bag <输入.bag> <输出.bag>

# CustomMsg → PointCloud2
rosrun livox_lidar_rosbag_converter custommsg_to_pointcloud2_bag <输入.bag> <输出.bag>
```

---

## 安装与使用（ROS 2）

> **ROS 2 用户请先切换分支**：  
> `git checkout ros2` 或克隆时指定分支 `-b ros2`。  
> 以下内容为简要指引，详细说明请见 `ros2` 分支下的 README。

### 1. 获取代码并编译

```bash
cd ~/ros2_ws/src
git clone -b ros2 https://github.com/FelixCooper1026/livox_lidar_rosbag_converter.git
cd ~/ros2_ws
colcon build --packages-select livox_lidar_rosbag_converter
source install/setup.bash
```

### 2. 使用方法（ROS 2）

#### 实时转换（节点）

| 转换方向                 | 命令                                                         |
|--------------------------|--------------------------------------------------------------|
| PointCloud2 → CustomMsg  | `ros2 run livox_lidar_rosbag_converter pointcloud2_to_custommsg` |
| CustomMsg → PointCloud2  | `ros2 run livox_lidar_rosbag_converter custommsg_to_pointcloud2` |

#### 直接转换 rosbag2 目录

```bash
# PointCloud2 → CustomMsg
ros2 run livox_lidar_rosbag_converter pointcloud2_to_custommsg_bag <输入目录> <输出目录>

# CustomMsg → PointCloud2
ros2 run livox_lidar_rosbag_converter custommsg_to_pointcloud2_bag <输入目录> <输出目录>
```

> 注意：输出目录必须为**尚不存在的路径**，由 `rosbag2` 自动创建。

---

## 话题说明（通用）

### 实时转换

| 转换方向                 | 订阅话题               | 发布话题                     |
|--------------------------|------------------------|------------------------------|
| PointCloud2 → CustomMsg  | `/livox/lidar` (PointCloud2) | `/livox/lidar_custommsg` (CustomMsg) |
| CustomMsg → PointCloud2  | `/livox/lidar` (CustomMsg)   | `/livox/lidar_pointcloud2` (PointCloud2) |

### rosbag 直接转换

- 程序自动识别输入 bag 中的第一个有效话题（PointCloud2 或 CustomMsg），转换后仍以同名话题 `/livox/lidar` 写入输出 bag。  
- 其余话题及消息时间戳将原样保留。

---

## 许可证

本项目使用 [MIT License](LICENSE)。

---

## 问题反馈

如有问题或建议，欢迎提交 [GitHub Issue](https://github.com/FelixCooper1026/livox_lidar_rosbag_converter/issues)。