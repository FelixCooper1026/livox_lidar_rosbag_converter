#include "custommsg_to_pointcloud2.hpp"
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

CustomMsgToPointCloud2::CustomMsgToPointCloud2(ros::NodeHandle& nh, const std::string& input_topic, 
                                              const std::string& output_topic)
    : input_topic_(input_topic), output_topic_(output_topic) {
  // 订阅CustomMsg格式的点云数据
  custommsg_subscription_ = nh.subscribe<livox_ros_driver2::CustomMsg>(
    input_topic_,
    10,
    &CustomMsgToPointCloud2::callbackCustomMsg, this
  );

  // 发布PointCloud2格式的点云数据
  pointcloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(
    output_topic_,
    10
  );
}

void CustomMsgToPointCloud2::callbackCustomMsg(const livox_ros_driver2::CustomMsgConstPtr& msg) {
  // 创建自定义点云对象
  pcl::PointCloud<PointXYZRTLT> cloud;
  cloud.reserve(msg->point_num);
  
  // 计算时间基数
  double timebase_sec = msg->timebase;

  // 转换每个点
  for (const auto& point : msg->points) {
    PointXYZRTLT pcl_point;
    pcl_point.x = point.x;
    pcl_point.y = point.y;
    pcl_point.z = point.z;
    pcl_point.intensity = static_cast<float>(point.reflectivity);
    pcl_point.tag = point.tag;
    pcl_point.line = point.line;
    // 计算绝对时间戳：timebase + offset_time
    pcl_point.timestamp = timebase_sec + point.offset_time;
    cloud.push_back(pcl_point);
  }

  // 转换为PointCloud2消息
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header = msg->header;

  // 发布消息
  pointcloud_publisher_.publish(cloud_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "custommsg_to_pointcloud2");
  ros::NodeHandle nh;
  
  CustomMsgToPointCloud2 converter(nh);
  ros::spin();
  return 0;
}