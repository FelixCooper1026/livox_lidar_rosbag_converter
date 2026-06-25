#include "custommsg_to_pointcloud2.hpp"
#include "livox_conversion.hpp"

#include <exception>

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
  try {
    pointcloud_publisher_.publish(livox_lidar_rosbag_converter::ToPointCloud2(*msg));
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("CustomMsg to PointCloud2 failed"
                     << "\n  input_topic: " << input_topic_
                     << "\n  output_topic: " << output_topic_
                     << "\n  message: " << livox_lidar_rosbag_converter::DescribeCustomMsg(*msg)
                     << "\n  details:\n" << e.what());
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "custommsg_to_pointcloud2");
  ros::NodeHandle nh;
  
  CustomMsgToPointCloud2 converter(nh);
  ros::spin();
  return 0;
}
