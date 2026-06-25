#ifndef POINTCLOUD2_TO_CUSTOMMSG_BAG_HPP
#define POINTCLOUD2_TO_CUSTOMMSG_BAG_HPP

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

class PointCloud2ToCustomMsgBag {
public:
  PointCloud2ToCustomMsgBag(const std::string & input_bag_path, const std::string & output_bag_path);

  bool convert();

private:
  livox_ros_driver2::msg::CustomMsg convertPointCloud2ToCustomMsg(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);

  std::string input_bag_path_;
  std::string output_bag_path_;
};

#endif  // POINTCLOUD2_TO_CUSTOMMSG_BAG_HPP
