#ifndef CUSTOMMSG_TO_POINTCLOUD2_BAG_HPP
#define CUSTOMMSG_TO_POINTCLOUD2_BAG_HPP

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

class CustomMsgToPointCloud2Bag {
public:
  CustomMsgToPointCloud2Bag(const std::string & input_bag_path, const std::string & output_bag_path);

  bool convert();

private:
  sensor_msgs::msg::PointCloud2 convertCustomMsgToPointCloud2(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg);

  std::string input_bag_path_;
  std::string output_bag_path_;
};

#endif  // CUSTOMMSG_TO_POINTCLOUD2_BAG_HPP
