#include "pointcloud2_to_custommsg.hpp"
#include "livox_conversion.hpp"

#include <exception>

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
  try {
    custommsg_publisher_.publish(livox_lidar_rosbag_converter::ToCustomMsg(*msg));
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("PointCloud2 to CustomMsg failed"
                     << "\n  input_topic: " << input_topic_
                     << "\n  output_topic: " << output_topic_
                     << "\n  details:\n" << e.what());
  }
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
