#include "pointcloud2_to_custommsg_bag.hpp"
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
        
        // 检查是否存在PointCloud2格式的Livox话题
        bool has_livox_topic = false;
        std::string livox_topic_name = "/livox/lidar";
        
        // 首先检查是否存在/livox/lidar话题
        for (const auto& conn : connections) {
            if (conn->topic == livox_topic_name && conn->datatype == "sensor_msgs/PointCloud2") {
                has_livox_topic = true;
                break;
            }
        }
        
        // 如果没有找到，查找任何PointCloud2类型的话题
        if (!has_livox_topic) {
            for (const auto& conn : connections) {
                if (conn->datatype == "sensor_msgs/PointCloud2") {
                    livox_topic_name = conn->topic;
                    has_livox_topic = true;
                    std::cout << "找到PointCloud2话题: " << livox_topic_name << std::endl;
                    break;
                }
            }
        }
        
        if (!has_livox_topic) {
            throw std::runtime_error("错误：输入bag文件中没有找到PointCloud2格式的点云话题！");
        }

        // 创建进度条
        ProgressBar progress(full_view.size(), "Converting PointCloud2 to CustomMsg");

        // 遍历所有消息
        bool has_converted_points = false;
        for (const rosbag::MessageInstance& m : full_view) {
            std::string topic = m.getTopic();
            
            if (topic == livox_topic_name) {
                // 处理PointCloud2消息
                sensor_msgs::PointCloud2ConstPtr pointcloud_msg = 
                    m.instantiate<sensor_msgs::PointCloud2>();
                if (pointcloud_msg != nullptr) {
                    // 转换消息
                    livox_ros_driver2::CustomMsg custom_msg = 
                        convertPointCloud2ToCustomMsg(pointcloud_msg);
                    // 写入新的bag文件，保持原始话题名
                    output_bag.write(topic, m.getTime(), custom_msg);
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

/**
 * @brief 将PointCloud2消息转换为CustomMsg消息
 * @param msg PointCloud2格式的点云消息
 * @return 转换后的CustomMsg消息
 */
livox_ros_driver2::CustomMsg PointCloud2ToCustomMsgBag::convertPointCloud2ToCustomMsg(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // 创建自定义点云对象
    pcl::PointCloud<PointXYZRTLT> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // 如果没有点，返回空消息
    if (cloud.empty()) {
        livox_ros_driver2::CustomMsg empty_msg;
        empty_msg.header = msg->header;
        empty_msg.point_num = 0;
        return empty_msg;
    }

    // 创建CustomMsg消息
    livox_ros_driver2::CustomMsg custom_msg;
    custom_msg.header = msg->header;
    custom_msg.point_num = cloud.size();
    custom_msg.lidar_id = 0;  // 默认设备ID
    custom_msg.rsvd[0] = 0;
    custom_msg.rsvd[1] = 0;
    custom_msg.rsvd[2] = 0;
    
    // 获取第一个点的时间戳作为timebase
    double first_timestamp = cloud[0].timestamp;
    custom_msg.timebase = static_cast<uint64_t>(first_timestamp);
    
    custom_msg.points.resize(cloud.size());

    // 转换每个点
    for (size_t i = 0; i < cloud.size(); ++i) {
        const auto& pcl_point = cloud[i];
        auto& custom_point = custom_msg.points[i];
        
        custom_point.x = pcl_point.x;
        custom_point.y = pcl_point.y;
        custom_point.z = pcl_point.z;
        custom_point.reflectivity = static_cast<uint8_t>(pcl_point.intensity);
        custom_point.tag = pcl_point.tag;
        custom_point.line = pcl_point.line;
        
        // 计算相对于timebase的偏移时间
        double point_time = pcl_point.timestamp;
        double offset_sec = point_time - first_timestamp;
        custom_point.offset_time = static_cast<uint32_t>(offset_sec);
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