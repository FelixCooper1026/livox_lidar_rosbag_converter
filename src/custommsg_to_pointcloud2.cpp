#include "custommsg_to_pointcloud2.hpp"
#include "livox_conversion.hpp"

#include <exception>
#include <functional>
#include <memory>

CustomMsgToPointCloud2::CustomMsgToPointCloud2(
  const rclcpp::NodeOptions & options, const std::string & input_topic, const std::string & output_topic)
  : rclcpp::Node("custommsg_to_pointcloud2", options), input_topic_(input_topic), output_topic_(output_topic)
{
  using std::placeholders::_1;
  custommsg_subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
    input_topic_, rclcpp::QoS(10), std::bind(&CustomMsgToPointCloud2::callbackCustomMsg, this, _1));
  pointcloud_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::QoS(10));
}

void CustomMsgToPointCloud2::callbackCustomMsg(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg)
{
  try {
    pointcloud_publisher_->publish(livox_lidar_rosbag_converter::ToPointCloud2(*msg));
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "CustomMsg to PointCloud2 failed"
        << "\n  input_topic: " << input_topic_
        << "\n  output_topic: " << output_topic_
        << "\n  message: " << livox_lidar_rosbag_converter::DescribeCustomMsg(*msg)
        << "\n  details:\n" << e.what());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomMsgToPointCloud2>());
  rclcpp::shutdown();
  return 0;
}
