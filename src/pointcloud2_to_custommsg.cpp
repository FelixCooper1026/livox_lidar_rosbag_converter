#include "pointcloud2_to_custommsg.hpp"
#include "livox_conversion.hpp"

#include <exception>
#include <functional>
#include <memory>

PointCloud2ToCustomMsg::PointCloud2ToCustomMsg(
  const rclcpp::NodeOptions & options, const std::string & input_topic, const std::string & output_topic)
  : rclcpp::Node("pointcloud2_to_custommsg", options), input_topic_(input_topic), output_topic_(output_topic)
{
  using std::placeholders::_1;
  pointcloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, rclcpp::QoS(10), std::bind(&PointCloud2ToCustomMsg::callbackPointCloud, this, _1));
  custommsg_publisher_ =
    create_publisher<livox_ros_driver2::msg::CustomMsg>(output_topic_, rclcpp::QoS(10));
}

void PointCloud2ToCustomMsg::callbackPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  try {
    custommsg_publisher_->publish(livox_lidar_rosbag_converter::ToCustomMsg(*msg));
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "PointCloud2 to CustomMsg failed"
        << "\n  input_topic: " << input_topic_
        << "\n  output_topic: " << output_topic_
        << "\n  details:\n" << e.what());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloud2ToCustomMsg>());
  rclcpp::shutdown();
  return 0;
}
