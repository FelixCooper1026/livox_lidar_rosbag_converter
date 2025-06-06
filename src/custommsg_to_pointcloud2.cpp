#include "custommsg_to_pointcloud2.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

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
  // 创建PCL点云对象
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.reserve(msg->point_num);

  // 转换每个点
  for (const auto& point : msg->points) {
    pcl::PointXYZI pcl_point;
    pcl_point.x = point.x;
    pcl_point.y = point.y;
    pcl_point.z = point.z;
    pcl_point.intensity = point.reflectivity;
    cloud.push_back(pcl_point);
  }

  // 转换为PointCloud2消息
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header = msg->header;

  // 发布消息
  pointcloud_publisher_.publish(cloud_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "custommsg_to_pointcloud2");
  ros::NodeHandle nh;
  
  CustomMsgToPointCloud2 converter(nh);
  ros::spin();
  return 0;
} 