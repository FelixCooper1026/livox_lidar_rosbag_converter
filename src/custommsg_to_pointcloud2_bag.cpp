#include "custommsg_to_pointcloud2_bag.hpp"
#include "bag_diagnostics.hpp"
#include "livox_conversion.hpp"
#include "progress_bar.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

void ReplaceOutputBag(const std::string& temporary_path, const std::string& output_path) {
    errno = 0;
    if (std::remove(output_path.c_str()) != 0 && errno != ENOENT) {
        throw std::runtime_error("Failed to replace output bag: " + output_path + ": " + std::strerror(errno));
    }

    errno = 0;
    if (std::rename(temporary_path.c_str(), output_path.c_str()) != 0) {
        throw std::runtime_error("Failed to move converted bag to output path: " + output_path + ": " +
                                 std::strerror(errno));
    }
}

}  // namespace

CustomMsgToPointCloud2Bag::CustomMsgToPointCloud2Bag(const std::string& input_bag_path,
                                                     const std::string& output_bag_path,
                                                     const std::string& topic_name)
    : input_bag_path_(input_bag_path), output_bag_path_(output_bag_path), topic_name_(topic_name) {}

bool CustomMsgToPointCloud2Bag::convert() {
    const std::string temporary_output_bag_path = output_bag_path_ + ".tmp";

    try {
        rosbag::Bag input_bag;
        input_bag.open(input_bag_path_, rosbag::bagmode::Read);

        rosbag::View full_view(input_bag);
        const std::vector<const rosbag::ConnectionInfo*> connections = full_view.getConnections();

        bool has_topic = false;
        for (const auto& conn : connections) {
            if (conn->topic == topic_name_ && conn->datatype == "livox_ros_driver2/CustomMsg") {
                has_topic = true;
                break;
            }
        }

        if (!has_topic) {
            throw std::runtime_error(livox_lidar_rosbag_converter::MissingTopicMessage(
                input_bag_path_, topic_name_, "livox_ros_driver2/CustomMsg", connections));
        }

        ProgressBar progress(full_view.size(), "Converting CustomMsg to PointCloud2");

        rosbag::Bag output_bag;
        output_bag.open(temporary_output_bag_path, rosbag::bagmode::Write);

        bool has_converted_points = false;
        for (const rosbag::MessageInstance& m : full_view) {
            const std::string topic = m.getTopic();

            if (topic == topic_name_) {
                const livox_ros_driver2::CustomMsgConstPtr custom_msg =
                    m.instantiate<livox_ros_driver2::CustomMsg>();
                if (custom_msg == nullptr) {
                    throw std::runtime_error(livox_lidar_rosbag_converter::MessageConversionFailure(
                        "message type mismatch", input_bag_path_, topic, m.getDataType(), m.getTime(),
                        "Requested topic message cannot be instantiated as livox_ros_driver2/CustomMsg"));
                }

                sensor_msgs::PointCloud2 pointcloud_msg;
                try {
                    pointcloud_msg = convertCustomMsgToPointCloud2(custom_msg);
                } catch (const std::exception& e) {
                    throw std::runtime_error(livox_lidar_rosbag_converter::MessageConversionFailure(
                        "message conversion failed", input_bag_path_, topic, m.getDataType(), m.getTime(), e.what()));
                }
                output_bag.write(topic, m.getTime(), pointcloud_msg);
                has_converted_points = true;
            } else {
                output_bag.write(topic, m.getTime(), m);
            }

            progress.update();
        }

        progress.finish();
        input_bag.close();
        output_bag.close();

        if (!has_converted_points) {
            throw std::runtime_error("No valid CustomMsg messages were converted");
        }

        ReplaceOutputBag(temporary_output_bag_path, output_bag_path_);

        std::cout << "Conversion completed: " << output_bag_path_ << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::remove(temporary_output_bag_path.c_str());
        std::cerr << "Conversion failed"
                  << "\n  output: " << output_bag_path_
                  << "\n" << e.what() << std::endl;
        return false;
    }
}

sensor_msgs::PointCloud2 CustomMsgToPointCloud2Bag::convertCustomMsgToPointCloud2(
    const livox_ros_driver2::CustomMsgConstPtr& msg) {
    return livox_lidar_rosbag_converter::ToPointCloud2(*msg);
}

int main(int argc, char** argv) {
    if (argc != 3 && argc != 4) {
        std::cout << "Usage: " << argv[0] << " <input.bag> <output.bag> [topic]" << std::endl;
        return 1;
    }

    ros::init(argc, argv, "custommsg_to_pointcloud2_bag");

    const std::string topic_name = argc == 4 ? argv[3] : "/livox/lidar";
    CustomMsgToPointCloud2Bag converter(argv[1], argv[2], topic_name);
    if (!converter.convert()) {
        return 1;
    }

    return 0;
}
