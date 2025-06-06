#include "bag_converter.hpp"
#include <iostream>

/**
 * @brief 构造函数实现
 * @param input_bag_path 输入bag文件路径
 * @param output_bag_path 输出bag文件路径
 */
BagConverter::BagConverter(const std::string& input_bag_path, const std::string& output_bag_path)
    : input_bag_path_(input_bag_path), output_bag_path_(output_bag_path) {}

/**
 * @brief 执行bag文件转换
 * @return 转换是否成功
 */
bool BagConverter::convert() {
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
livox_ros_driver2::CustomMsg BagConverter::convertPointCloud2ToCustomMsg(const sensor_msgs::PointCloud2ConstPtr& msg) {
    livox_ros_driver2::CustomMsg custom_msg;
    std::vector<livox_ros_driver2::CustomPoint> custom_points;

    // 设置消息头信息
    custom_msg.header = msg->header;
    custom_msg.timebase = msg->header.stamp.toNSec();
    custom_msg.point_num = msg->width;
    custom_msg.lidar_id = 0;
    
    // 获取点云数据指针
    const uint8_t* data_ptr = msg->data.data();
    
    // 遍历所有点
    for (unsigned int i = 0; i < msg->width; i++) {
        // 计算当前点的数据指针位置
        const uint8_t* point_ptr = data_ptr + i * msg->point_step;
        float x, y, z, intensity;
        uint8_t tag, line;
        
        // 从点云数据中提取各个字段
        std::memcpy(&x, point_ptr + 0, sizeof(float));        // x坐标
        std::memcpy(&y, point_ptr + 4, sizeof(float));        // y坐标
        std::memcpy(&z, point_ptr + 8, sizeof(float));        // z坐标
        std::memcpy(&intensity, point_ptr + 12, sizeof(float)); // 强度值
        
        // 限制强度值在0-255范围内
        intensity = std::max(0.0f, std::min(intensity, 255.0f));
        tag = *(point_ptr + 16);  // 标签
        line = *(point_ptr + 17); // 线号
        
        // 创建Livox自定义点
        livox_ros_driver2::CustomPoint custom_point;
        custom_point.offset_time = 0;  // 时间偏移
        custom_point.x = x;
        custom_point.y = y;
        custom_point.z = z;
        custom_point.reflectivity = static_cast<uint8_t>(intensity + 0.5f);  // 四舍五入强度值
        custom_point.tag = tag;
        custom_point.line = line;
        custom_points.push_back(custom_point);
    }
    
    custom_msg.points = custom_points;
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
    ros::init(argc, argv, "bag_converter");
    
    // 创建转换器实例并执行转换
    BagConverter converter(argv[1], argv[2]);
    if (!converter.convert()) {
        return 1;
    }
    
    return 0;
} 