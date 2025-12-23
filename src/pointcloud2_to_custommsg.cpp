#include "pointcloud2_to_custommsg.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types_conversion.h>

// 自定义点类型，匹配Livox PointXYZRTLT格式
struct PointXYZRTLT
{
  PCL_ADD_POINT4D;      // XYZ
  float intensity;      // 反射强度
  uint8_t tag;          // livox标签
  uint8_t line;         // 激光线号
  double timestamp;     // 时间戳
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRTLT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (uint8_t, tag, tag)
  (uint8_t, line, line)
  (double, timestamp, timestamp)
)

/**
 * @brief 构造函数实现
 * @param nh ROS节点句柄
 * @param input_topic 输入话题名
 * @param output_topic 输出话题名
 */
PointCloud2ToCustomMsg::PointCloud2ToCustomMsg(ros::NodeHandle& nh, const std::string& input_topic, 
                                              const std::string& output_topic)
    : input_topic_(input_topic), output_topic_(output_topic) {
  // 订阅PointCloud2格式的点云数据
  pointcloud_subscription_ = nh.subscribe<sensor_msgs::PointCloud2>(
    input_topic_,
    10,
    &PointCloud2ToCustomMsg::callbackPointCloud, this
  );

  // 发布CustomMsg格式的点云数据
  custommsg_publisher_ = nh.advertise<livox_ros_driver2::CustomMsg>(
    output_topic_,
    10
  );
}

/**
 * @brief 点云数据回调函数实现
 * @param msg PointCloud2格式的点云消息
 */
void PointCloud2ToCustomMsg::callbackPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg) {
  // 创建自定义点云对象
  pcl::PointCloud<PointXYZRTLT> cloud;
  pcl::fromROSMsg(*msg, cloud);

  // 如果没有点，直接返回
  if (cloud.empty()) {
    ROS_WARN("Empty point cloud received");
    return;
  }

  // 创建CustomMsg消息
  livox_ros_driver2::CustomMsg custom_msg;
  custom_msg.header = msg->header;
  custom_msg.point_num = cloud.size();
  custom_msg.lidar_id = 0;  // 默认设备ID
  custom_msg.rsvd[0] = 0;
  custom_msg.rsvd[1] = 0;
  custom_msg.rsvd[2] = 0;
  
  // 获取第一个点的时间戳作为timebase
  double first_timestamp = cloud[0].timestamp;
  custom_msg.timebase = static_cast<uint64_t>(first_timestamp);
  
  custom_msg.points.resize(cloud.size());

  // 转换每个点
  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto& pcl_point = cloud[i];
    auto& custom_point = custom_msg.points[i];
    
    custom_point.x = pcl_point.x;
    custom_point.y = pcl_point.y;
    custom_point.z = pcl_point.z;
    custom_point.reflectivity = static_cast<uint8_t>(pcl_point.intensity);
    custom_point.tag = pcl_point.tag;
    custom_point.line = pcl_point.line;
    
    // 计算相对于timebase的偏移时间
    double point_time = pcl_point.timestamp;
    double offset_sec = point_time - first_timestamp;
    custom_point.offset_time = static_cast<uint32_t>(offset_sec);
  }

  // 发布消息
  custommsg_publisher_.publish(custom_msg);
}

/**
 * @brief 主函数
 */
int main(int argc, char** argv) {
  // 初始化ROS节点
  ros::init(argc, argv, "pointcloud2_to_custommsg");
  ros::NodeHandle nh;
  
  // 创建转换器实例，使用默认话题名
  PointCloud2ToCustomMsg converter(nh);
  
  // 进入ROS消息循环
  ros::spin();
  return 0;
}