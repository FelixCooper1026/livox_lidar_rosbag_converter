#ifndef LIDAR_CONVERTER_HPP
#define LIDAR_CONVERTER_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver2/CustomMsg.h>  // Livox ROS驱动2的自定义消息类型
#include <string>

/**
 * @brief Livox激光雷达数据格式转换器类
 * 
 * 该类用于将PointCloud2格式的点云数据转换为Livox自定义消息格式
 * 支持实时转换和bag文件转换两种模式
 */
class LidarConverter {
public:
  /**
   * @brief 构造函数
   * @param nh ROS节点句柄
   * @param input_topic 输入话题名，默认为"/livox/lidar"
   * @param output_topic 输出话题名，默认为"/livox/lidar_custommsg"
   */
  LidarConverter(ros::NodeHandle& nh, const std::string& input_topic = "/livox/lidar", 
                const std::string& output_topic = "/livox/lidar_custommsg");
  
private:
  /**
   * @brief 点云数据回调函数
   * @param msg PointCloud2格式的点云消息
   */
  void callbackPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg);
  
  ros::Subscriber pointcloud_subscription_;  // 点云数据订阅者
  ros::Publisher custommsg_publisher_;       // 自定义消息发布者
  std::string input_topic_;                  // 输入话题名
  std::string output_topic_;                 // 输出话题名
};

#endif // LIDAR_CONVERTER_HPP
