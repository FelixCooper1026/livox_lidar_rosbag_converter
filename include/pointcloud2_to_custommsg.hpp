#ifndef POINTCLOUD2_TO_CUSTOMMSG_HPP
#define POINTCLOUD2_TO_CUSTOMMSG_HPP

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

class PointCloud2ToCustomMsg : public rclcpp::Node {
public:
  explicit PointCloud2ToCustomMsg(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    const std::string & input_topic = "/livox/lidar",
    const std::string & output_topic = "/livox/lidar_custommsg");

private:
  void callbackPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custommsg_publisher_;
  std::string input_topic_;
  std::string output_topic_;
};

#endif  // POINTCLOUD2_TO_CUSTOMMSG_HPP
