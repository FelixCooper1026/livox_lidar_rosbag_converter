#include "lidar_converter.hpp"

/**
 * @brief 构造函数实现
 * @param nh ROS节点句柄
 * @param input_topic 输入话题名
 * @param output_topic 输出话题名
 */
LidarConverter::LidarConverter(ros::NodeHandle& nh, const std::string& input_topic, 
                             const std::string& output_topic)
    : input_topic_(input_topic), output_topic_(output_topic) {
  // 订阅PointCloud2格式的点云数据
  pointcloud_subscription_ = nh.subscribe<sensor_msgs::PointCloud2>(
    input_topic_,
    10,  // 消息队列大小
    &LidarConverter::callbackPointCloud, this
  );

  // 发布Livox自定义消息格式的点云数据
  custommsg_publisher_ = nh.advertise<livox_ros_driver2::CustomMsg>(
    output_topic_,
    10  // 消息队列大小
  );
}

/**
 * @brief 点云数据回调函数实现
 * @param msg PointCloud2格式的点云消息
 */
void LidarConverter::callbackPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg) {
  livox_ros_driver2::CustomMsg custom_msg;
  std::vector<livox_ros_driver2::CustomPoint> custom_points;

  // 设置消息头信息
  custom_msg.header = msg->header;
  custom_msg.timebase = msg->header.stamp.toNSec();
  custom_msg.point_num = msg->width;
  custom_msg.lidar_id = 0;
  
  // 获取点云数据指针
  const uint8_t* data_ptr = msg->data.data();
  
  // 遍历所有点
  for (unsigned int i = 0; i < msg->width; i++) {
    // 计算当前点的数据指针位置
    const uint8_t* point_ptr = data_ptr + i * msg->point_step;
    float x, y, z, intensity;
    uint8_t tag, line;
    
    // 从点云数据中提取各个字段
    std::memcpy(&x, point_ptr + 0, sizeof(float));        // x坐标
    std::memcpy(&y, point_ptr + 4, sizeof(float));        // y坐标
    std::memcpy(&z, point_ptr + 8, sizeof(float));        // z坐标
    std::memcpy(&intensity, point_ptr + 12, sizeof(float)); // 强度值
    
    // 限制强度值在0-255范围内
    intensity = std::max(0.0f, std::min(intensity, 255.0f));
    tag = *(point_ptr + 16);  // 标签
    line = *(point_ptr + 17); // 线号
    
    // 创建Livox自定义点
    livox_ros_driver2::CustomPoint custom_point;
    custom_point.offset_time = 0;  // 时间偏移
    custom_point.x = x;
    custom_point.y = y;
    custom_point.z = z;
    custom_point.reflectivity = static_cast<uint8_t>(intensity + 0.5f);  // 四舍五入强度值
    custom_point.tag = tag;
    custom_point.line = line;
    custom_points.push_back(custom_point);
  }
  
  // 设置点云数据并发布
  custom_msg.points = custom_points;
  custommsg_publisher_.publish(custom_msg);
}

/**
 * @brief 主函数
 */
int main(int argc, char** argv) {
  // 初始化ROS节点
  ros::init(argc, argv, "lidar_converter");
  ros::NodeHandle nh;
  
  // 创建转换器实例，使用默认话题名
  LidarConverter converter(nh, "/livox/lidar", "/livox/lidar_custommsg");
  
  // 进入ROS消息循环
  ros::spin();
  return 0;
}