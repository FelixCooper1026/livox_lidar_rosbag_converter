#include "pointcloud2_to_custommsg_bag.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

/**
 * @brief 构造函数实现
 * @param input_bag_path 输入bag文件路径
 * @param output_bag_path 输出bag文件路径
 */
PointCloud2ToCustomMsgBag::PointCloud2ToCustomMsgBag(const std::string& input_bag_path, const std::string& output_bag_path)
    : input_bag_path_(input_bag_path), output_bag_path_(output_bag_path) {}

/**
 * @brief 执行bag文件转换
 * @return 转换是否成功
 */
bool PointCloud2ToCustomMsgBag::convert() {
    try {
        // 打开输入bag文件
        rosbag::Bag input_bag;
        input_bag.open(input_bag_path_, rosbag::bagmode::Read);

        // 创建输出bag文件
        rosbag::Bag output_bag;
        output_bag.open(output_bag_path_, rosbag::bagmode::Write);

        // 获取所有话题
        rosbag::View full_view(input_bag);
        std::vector<const rosbag::ConnectionInfo*> connections = full_view.getConnections();
        
        // 遍历所有消息
        for (const rosbag::MessageInstance& m : full_view) {
            std::string topic = m.getTopic();
            
            if (topic == "/livox/lidar") {
                // 处理PointCloud2消息
                sensor_msgs::PointCloud2ConstPtr pointcloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
                if (pointcloud_msg != nullptr) {
                    // 转换消息
                    livox_ros_driver2::CustomMsg custom_msg = convertPointCloud2ToCustomMsg(pointcloud_msg);
                    // 写入新的bag文件，保持原始话题名
                    output_bag.write(topic, m.getTime(), custom_msg);
                }
            } else {
                // 直接复制其他话题的消息
                output_bag.write(topic, m.getTime(), m);
            }
        }

        // 关闭bag文件
        input_bag.close();
        output_bag.close();
        
        std::cout << "转换完成！输出文件: " << output_bag_path_ << std::endl;
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "转换过程中发生错误: " << e.what() << std::endl;
        return false;
    }
}

/**
 * @brief 将PointCloud2消息转换为CustomMsg消息
 * @param msg PointCloud2格式的点云消息
 * @return 转换后的CustomMsg消息
 */
livox_ros_driver2::CustomMsg PointCloud2ToCustomMsgBag::convertPointCloud2ToCustomMsg(const sensor_msgs::PointCloud2ConstPtr& msg) {
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

    return custom_msg;
}

/**
 * @brief 主函数
 */
int main(int argc, char** argv) {
    // 检查命令行参数
    if (argc != 3) {
        std::cout << "用法: " << argv[0] << " <输入bag文件路径> <输出bag文件路径>" << std::endl;
        return 1;
    }

    // 初始化ROS节点
    ros::init(argc, argv, "pointcloud2_to_custommsg_bag");
    
    // 创建转换器实例并执行转换
    PointCloud2ToCustomMsgBag converter(argv[1], argv[2]);
    if (!converter.convert()) {
        return 1;
    }
    
    return 0;
} 