#include "custommsg_to_pointcloud2_bag.hpp"
#include "livox_conversion.hpp"
#include "progress_bar.hpp"

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include <dirent.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sys/stat.h>
#include <unistd.h>

namespace {

constexpr const char * kPc2Type = "sensor_msgs/msg/PointCloud2";
constexpr const char * kCustomType = "livox_ros_driver2/msg/CustomMsg";

void remove_directory_recursive(const std::string & path)
{
  DIR * dir = opendir(path.c_str());
  if (dir == nullptr) {
    return;
  }

  while (dirent * entry = readdir(dir)) {
    const std::string name = entry->d_name;
    if (name == "." || name == "..") {
      continue;
    }

    const std::string child_path = path + "/" + name;
    struct stat stat_buffer {};
    if (lstat(child_path.c_str(), &stat_buffer) != 0) {
      continue;
    }

    if (S_ISDIR(stat_buffer.st_mode)) {
      remove_directory_recursive(child_path);
    } else {
      std::remove(child_path.c_str());
    }
  }

  closedir(dir);
  rmdir(path.c_str());
}

void move_output_directory(const std::string & temporary_path, const std::string & output_path)
{
  errno = 0;
  if (std::rename(temporary_path.c_str(), output_path.c_str()) != 0) {
    throw std::runtime_error("无法生成输出目录: " + output_path + ": " + std::strerror(errno));
  }
}

bool find_livox_custom_topic(const rosbag2_storage::BagMetadata & meta, std::string & out_topic)
{
  const std::string preferred = "/livox/lidar";
  bool preferred_wrong_type = false;

  for (const auto & ti : meta.topics_with_message_count) {
    if (ti.topic_metadata.name != preferred) {
      continue;
    }
    if (ti.topic_metadata.type == kCustomType) {
      out_topic = preferred;
      return true;
    }
    preferred_wrong_type = true;
    break;
  }

  if (preferred_wrong_type) {
    std::cout << "警告：输入 bag 中的 /livox/lidar 不是 CustomMsg，尝试查找其他 CustomMsg 话题..." << std::endl;
  }

  for (const auto & ti : meta.topics_with_message_count) {
    if (ti.topic_metadata.type == kCustomType) {
      out_topic = ti.topic_metadata.name;
      if (out_topic != preferred) {
        std::cout << "找到 CustomMsg 话题: " << out_topic << std::endl;
      }
      return true;
    }
  }
  return false;
}

void register_topics_custom_to_pc2(
  rosbag2_cpp::Writer & writer, const rosbag2_storage::BagMetadata & meta, const std::string & livox_topic)
{
  for (const auto & ti : meta.topics_with_message_count) {
    rosbag2_storage::TopicMetadata tm = ti.topic_metadata;
    if (tm.name == livox_topic) {
      tm.type = kPc2Type;
    }
    writer.create_topic(tm);
  }
}

}  // namespace

CustomMsgToPointCloud2Bag::CustomMsgToPointCloud2Bag(
  const std::string & input_bag_path, const std::string & output_bag_path)
  : input_bag_path_(input_bag_path), output_bag_path_(output_bag_path)
{
}

sensor_msgs::msg::PointCloud2 CustomMsgToPointCloud2Bag::convertCustomMsgToPointCloud2(
  const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg)
{
  return livox_lidar_rosbag_converter::ToPointCloud2(*msg);
}

bool CustomMsgToPointCloud2Bag::convert()
{
  const std::string temporary_output_bag_path = output_bag_path_ + ".tmp";

  try {
    rosbag2_cpp::Reader reader;
    reader.open(input_bag_path_);

    const rosbag2_storage::BagMetadata & meta = reader.get_metadata();
    std::string livox_topic;
    if (!find_livox_custom_topic(meta, livox_topic)) {
      throw std::runtime_error("错误：输入 bag 中未找到 livox_ros_driver2/msg/CustomMsg 话题！");
    }

    rosbag2_cpp::Writer writer;
    writer.open(temporary_output_bag_path);

    register_topics_custom_to_pc2(writer, meta, livox_topic);

    int total = static_cast<int>(std::min<uint64_t>(meta.message_count, static_cast<uint64_t>(INT_MAX)));
    if (total <= 0) {
      total = 1;
    }
    ProgressBar progress(total, "Converting CustomMsg to PointCloud2");

    rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> serialization_custom;

    bool has_converted_points = false;
    while (reader.has_next()) {
      auto bag_msg = reader.read_next();
      progress.update();

      if (bag_msg->topic_name == livox_topic) {
        rclcpp::SerializedMessage extracted(*bag_msg->serialized_data);
        auto custom = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
        serialization_custom.deserialize_message(&extracted, custom.get());
        sensor_msgs::msg::PointCloud2 pc2 = convertCustomMsgToPointCloud2(custom);
        writer.write(pc2, livox_topic, rclcpp::Time(bag_msg->time_stamp));
        has_converted_points = true;
      } else {
        writer.write(bag_msg);
      }
    }

    progress.finish();
    writer.close();
    reader.close();

    if (!has_converted_points) {
      throw std::runtime_error("错误：转换过程中没有找到有效的点云数据！");
    }

    move_output_directory(temporary_output_bag_path, output_bag_path_);

    std::cout << "转换完成！输出目录: " << output_bag_path_ << std::endl;
    return true;
  } catch (const std::exception & e) {
    remove_directory_recursive(temporary_output_bag_path);
    std::cerr << "转换过程中发生错误: " << e.what() << std::endl;
    return false;
  }
}

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cout << "用法: " << argv[0] << " <输入 bag 目录路径> <输出 bag 目录路径>" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);

  CustomMsgToPointCloud2Bag converter(argv[1], argv[2]);
  const bool ok = converter.convert();

  rclcpp::shutdown();
  return ok ? 0 : 1;
}
