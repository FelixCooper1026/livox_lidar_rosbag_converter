#include "custommsg_to_pointcloud2_bag.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

CustomMsgToPointCloud2Bag::CustomMsgToPointCloud2Bag(const std::string& input_bag_path, 
                                                    const std::string& output_bag_path)
    : input_bag_path_(input_bag_path), output_bag_path_(output_bag_path) {}

bool CustomMsgToPointCloud2Bag::convert() {
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
        
        // 检查是否存在/livox/lidar话题
        bool has_livox_topic = false;
        for (const auto& conn : connections) {
            if (conn->topic == "/livox/lidar") {
                has_livox_topic = true;
                // 检查话题类型
                if (conn->datatype != "livox_ros_driver2/CustomMsg") {
                    throw std::runtime_error("错误：输入bag文件中的/livox/lidar话题不是CustomMsg格式！");
                }
                break;
            }
        }
        
        if (!has_livox_topic) {
            throw std::runtime_error("错误：输入bag文件中没有找到/livox/lidar话题！");
        }

        // 遍历所有消息
        bool has_converted_points = false;
        for (const rosbag::MessageInstance& m : full_view) {
            std::string topic = m.getTopic();
            
            if (topic == "/livox/lidar") {
                // 处理CustomMsg消息
                livox_ros_driver2::CustomMsgConstPtr custom_msg = 
                    m.instantiate<livox_ros_driver2::CustomMsg>();
                if (custom_msg != nullptr) {
                    // 转换消息
                    sensor_msgs::PointCloud2 pointcloud_msg = 
                        convertCustomMsgToPointCloud2(custom_msg);
                    // 写入新的bag文件，保持原始话题名
                    output_bag.write(topic, m.getTime(), pointcloud_msg);
                    has_converted_points = true;
                }
            } else {
                // 直接复制其他话题的消息
                output_bag.write(topic, m.getTime(), m);
            }
        }

        // 关闭bag文件
        input_bag.close();
        output_bag.close();
        
        if (!has_converted_points) {
            throw std::runtime_error("错误：转换过程中没有找到有效的点云数据！");
        }
        
        std::cout << "转换完成！输出文件: " << output_bag_path_ << std::endl;
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "转换过程中发生错误: " << e.what() << std::endl;
        return false;
    }
}

sensor_msgs::PointCloud2 CustomMsgToPointCloud2Bag::convertCustomMsgToPointCloud2(
    const livox_ros_driver2::CustomMsgConstPtr& msg) {
    
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

    return cloud_msg;
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cout << "用法: " << argv[0] << " <输入bag文件路径> <输出bag文件路径>" << std::endl;
        return 1;
    }

    ros::init(argc, argv, "custommsg_to_pointcloud2_bag");
    
    CustomMsgToPointCloud2Bag converter(argv[1], argv[2]);
    if (!converter.convert()) {
        return 1;
    }
    
    return 0;
}