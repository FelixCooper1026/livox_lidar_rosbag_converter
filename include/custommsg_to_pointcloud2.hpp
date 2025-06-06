#ifndef CUSTOMMSG_TO_POINTCLOUD2_HPP
#define CUSTOMMSG_TO_POINTCLOUD2_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <string>

/**
 * @brief CustomMsg到PointCloud2格式转换器类
 * 
 * 该类用于将Livox自定义消息格式转换为PointCloud2格式
 * 支持实时转换模式
 */
class CustomMsgToPointCloud2 {
public:
  /**
   * @brief 构造函数
   * @param nh ROS节点句柄
   * @param input_topic 输入话题名，默认为"/livox/lidar"
   * @param output_topic 输出话题名，默认为"/livox/lidar_pointcloud2"
   */
  CustomMsgToPointCloud2(ros::NodeHandle& nh, const std::string& input_topic = "/livox/lidar", 
                        const std::string& output_topic = "/livox/lidar_pointcloud2");
  
private:
  /**
   * @brief 点云数据回调函数
   * @param msg CustomMsg格式的点云消息
   */
  void callbackCustomMsg(const livox_ros_driver2::CustomMsgConstPtr& msg);
  
  ros::Subscriber custommsg_subscription_;  // 自定义消息订阅者
  ros::Publisher pointcloud_publisher_;     // PointCloud2消息发布者
  std::string input_topic_;                 // 输入话题名
  std::string output_topic_;                // 输出话题名
};

#endif // CUSTOMMSG_TO_POINTCLOUD2_HPP 