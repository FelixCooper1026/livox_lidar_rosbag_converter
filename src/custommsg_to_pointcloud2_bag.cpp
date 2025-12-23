#include "custommsg_to_pointcloud2_bag.hpp"
#include "progress_bar.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <iostream>

// 自定义点类型，匹配Livox PointXYZRTLT格式
struct PointXYZRTLT
{
  PCL_ADD_POINT4D;      // XYZ
  float intensity;      // 反射强度
  uint8_t tag;          // livox标签
  uint8_t line;         // 激光线号
  double timestamp;     // 时间戳
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRTLT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (uint8_t, tag, tag)
  (uint8_t, line, line)
  (double, timestamp, timestamp)
)

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
        std::string livox_topic_name = "/livox/lidar";
        
        // 首先检查是否存在/livox/lidar话题
        for (const auto& conn : connections) {
            if (conn->topic == livox_topic_name) {
                has_livox_topic = true;
                // 检查话题类型
                if (conn->datatype != "livox_ros_driver2/CustomMsg") {
                    std::cout << "警告：输入bag文件中的/livox/lidar话题不是CustomMsg格式，尝试查找其他CustomMsg话题..." << std::endl;
                    has_livox_topic = false;
                }
                break;
            }
        }
        
        // 如果没有找到/livox/lidar，查找任何包含"CustomMsg"类型的话题
        if (!has_livox_topic) {
            for (const auto& conn : connections) {
                if (conn->datatype == "livox_ros_driver2/CustomMsg") {
                    livox_topic_name = conn->topic;
                    has_livox_topic = true;
                    std::cout << "找到CustomMsg话题: " << livox_topic_name << std::endl;
                    break;
                }
            }
        }
        
        if (!has_livox_topic) {
            throw std::runtime_error("错误：输入bag文件中没有找到CustomMsg格式的话题！");
        }

        // 创建进度条
        ProgressBar progress(full_view.size(), "Converting CustomMsg to PointCloud2");

        // 遍历所有消息
        bool has_converted_points = false;
        for (const rosbag::MessageInstance& m : full_view) {
            std::string topic = m.getTopic();
            
            if (topic == livox_topic_name) {
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
            progress.update();
        }

        // 完成进度条
        progress.finish();

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
    
    // 创建自定义点云对象
    pcl::PointCloud<PointXYZRTLT> cloud;
    cloud.reserve(msg->point_num);
    
    // 计算时间基数   
    double timebase_sec = msg->timebase;

    // 转换每个点
    for (const auto& point : msg->points) {
        PointXYZRTLT pcl_point;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = point.z;
        pcl_point.intensity = static_cast<float>(point.reflectivity);
        pcl_point.tag = point.tag;
        pcl_point.line = point.line;
        // 计算绝对时间戳：timebase + offset_time
        pcl_point.timestamp = timebase_sec + point.offset_time;
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