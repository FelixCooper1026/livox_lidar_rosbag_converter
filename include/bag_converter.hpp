#ifndef BAG_CONVERTER_HPP
#define BAG_CONVERTER_HPP

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <string>

/**
 * @brief ROS bag文件转换器类
 * 
 * 该类用于将包含PointCloud2格式点云数据的bag文件转换为CustomMsg格式
 * 同时保留bag文件中的其他话题数据
 */
class BagConverter {
public:
    /**
     * @brief 构造函数
     * @param input_bag_path 输入bag文件路径
     * @param output_bag_path 输出bag文件路径
     */
    BagConverter(const std::string& input_bag_path, const std::string& output_bag_path);

    /**
     * @brief 执行bag文件转换
     * @return 转换是否成功
     */
    bool convert();

private:
    /**
     * @brief 将PointCloud2消息转换为CustomMsg消息
     * @param msg PointCloud2格式的点云消息
     * @return 转换后的CustomMsg消息
     */
    livox_ros_driver2::CustomMsg convertPointCloud2ToCustomMsg(const sensor_msgs::PointCloud2ConstPtr& msg);
    
    std::string input_bag_path_;   // 输入bag文件路径
    std::string output_bag_path_;  // 输出bag文件路径
};

#endif // BAG_CONVERTER_HPP 