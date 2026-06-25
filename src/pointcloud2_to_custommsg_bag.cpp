#include "pointcloud2_to_custommsg_bag.hpp"
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

bool find_livox_pointcloud2_topic(const rosbag2_storage::BagMetadata & meta, std::string & out_topic)
{
  const std::string preferred = "/livox/lidar";
  for (const auto & ti : meta.topics_with_message_count) {
    if (ti.topic_metadata.name == preferred && ti.topic_metadata.type == kPc2Type) {
      out_topic = preferred;
      return true;
    }
  }
  for (const auto & ti : meta.topics_with_message_count) {
    if (ti.topic_metadata.type == kPc2Type) {
      out_topic = ti.topic_metadata.name;
      std::cout << "找到 PointCloud2 话题: " << out_topic << std::endl;
      return true;
    }
  }
  return false;
}

void register_topics_pc2_to_custom(
  rosbag2_cpp::Writer & writer, const rosbag2_storage::BagMetadata & meta, const std::string & livox_topic)
{
  for (const auto & ti : meta.topics_with_message_count) {
    rosbag2_storage::TopicMetadata tm = ti.topic_metadata;
    if (tm.name == livox_topic) {
      tm.type = kCustomType;
    }
    writer.create_topic(tm);
  }
}

}  // namespace

PointCloud2ToCustomMsgBag::PointCloud2ToCustomMsgBag(
  const std::string & input_bag_path, const std::string & output_bag_path)
  : input_bag_path_(input_bag_path), output_bag_path_(output_bag_path)
{
}

livox_ros_driver2::msg::CustomMsg PointCloud2ToCustomMsgBag::convertPointCloud2ToCustomMsg(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  return livox_lidar_rosbag_converter::ToCustomMsg(*msg);
}

bool PointCloud2ToCustomMsgBag::convert()
{
  const std::string temporary_output_bag_path = output_bag_path_ + ".tmp";

  try {
    rosbag2_cpp::Reader reader;
    reader.open(input_bag_path_);

    const rosbag2_storage::BagMetadata & meta = reader.get_metadata();
    std::string livox_topic;
    if (!find_livox_pointcloud2_topic(meta, livox_topic)) {
      throw std::runtime_error("错误：输入 bag 中未找到 sensor_msgs/msg/PointCloud2 点云话题！");
    }

    rosbag2_cpp::Writer writer;
    writer.open(temporary_output_bag_path);

    register_topics_pc2_to_custom(writer, meta, livox_topic);

    int total = static_cast<int>(std::min<uint64_t>(meta.message_count, static_cast<uint64_t>(INT_MAX)));
    if (total <= 0) {
      total = 1;
    }
    ProgressBar progress(total, "Converting PointCloud2 to CustomMsg");

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization_pc2;

    bool has_converted_points = false;
    while (reader.has_next()) {
      auto bag_msg = reader.read_next();
      progress.update();

      if (bag_msg->topic_name == livox_topic) {
        rclcpp::SerializedMessage extracted(*bag_msg->serialized_data);
        auto pc2 = std::make_shared<sensor_msgs::msg::PointCloud2>();
        serialization_pc2.deserialize_message(&extracted, pc2.get());
        livox_ros_driver2::msg::CustomMsg custom_msg = convertPointCloud2ToCustomMsg(pc2);
        writer.write(custom_msg, livox_topic, rclcpp::Time(bag_msg->time_stamp));
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

  PointCloud2ToCustomMsgBag converter(argv[1], argv[2]);
  const bool ok = converter.convert();

  rclcpp::shutdown();
  return ok ? 0 : 1;
}
