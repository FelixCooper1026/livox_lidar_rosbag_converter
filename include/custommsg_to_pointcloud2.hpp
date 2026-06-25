#ifndef CUSTOMMSG_TO_POINTCLOUD2_HPP
#define CUSTOMMSG_TO_POINTCLOUD2_HPP

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

class CustomMsgToPointCloud2 : public rclcpp::Node {
public:
  explicit CustomMsgToPointCloud2(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    const std::string & input_topic = "/livox/lidar",
    const std::string & output_topic = "/livox/lidar_pointcloud2");

private:
  void callbackCustomMsg(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg);

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr custommsg_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
  std::string input_topic_;
  std::string output_topic_;
};

#endif  // CUSTOMMSG_TO_POINTCLOUD2_HPP
