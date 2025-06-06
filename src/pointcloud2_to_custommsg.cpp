#include "pointcloud2_to_custommsg.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

/**
 * @brief 构造函数实现
 * @param nh ROS节点句柄
 * @param input_topic 输入话题名
 * @param output_topic 输出话题名
 */
PointCloud2ToCustomMsg::PointCloud2ToCustomMsg(ros::NodeHandle& nh, const std::string& input_topic, 
                                              const std::string& output_topic)
    : input_topic_(input_topic), output_topic_(output_topic) {
  // 订阅PointCloud2格式的点云数据
  pointcloud_subscription_ = nh.subscribe<sensor_msgs::PointCloud2>(
    input_topic_,
    10,
    &PointCloud2ToCustomMsg::callbackPointCloud, this
  );

  // 发布CustomMsg格式的点云数据
  custommsg_publisher_ = nh.advertise<livox_ros_driver2::CustomMsg>(
    output_topic_,
    10
  );
}

/**
 * @brief 点云数据回调函数实现
 * @param msg PointCloud2格式的点云消息
 */
void PointCloud2ToCustomMsg::callbackPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg) {
  // 创建PCL点云对象
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg(*msg, cloud);

  // 创建CustomMsg消息
  livox_ros_driver2::CustomMsg custom_msg;
  custom_msg.header = msg->header;
  custom_msg.point_num = cloud.size();
  custom_msg.points.resize(cloud.size());

  // 转换每个点
  for (size_t i = 0; i < cloud.size(); ++i) {
    custom_msg.points[i].x = cloud[i].x;
    custom_msg.points[i].y = cloud[i].y;
    custom_msg.points[i].z = cloud[i].z;
    custom_msg.points[i].reflectivity = cloud[i].intensity;
    custom_msg.points[i].tag = 0;
    custom_msg.points[i].line = 0;
  }

  // 发布消息
  custommsg_publisher_.publish(custom_msg);
}

/**
 * @brief 主函数
 */
int main(int argc, char** argv) {
  // 初始化ROS节点
  ros::init(argc, argv, "pointcloud2_to_custommsg");
  ros::NodeHandle nh;
  
  // 创建转换器实例，使用默认话题名
  PointCloud2ToCustomMsg converter(nh);
  
  // 进入ROS消息循环
  ros::spin();
  return 0;
}