#ifndef BAG_DIAGNOSTICS_HPP
#define BAG_DIAGNOSTICS_HPP

#include <ros/time.h>
#include <rosbag/view.h>

#include <sstream>
#include <string>
#include <vector>

namespace livox_lidar_rosbag_converter {

inline std::string DescribeBagConnections(const std::vector<const rosbag::ConnectionInfo*>& connections) {
  std::ostringstream stream;
  if (connections.empty()) {
    stream << "none";
    return stream.str();
  }

  for (size_t i = 0; i < connections.size(); ++i) {
    stream << "\n    - " << connections[i]->topic << " [" << connections[i]->datatype << "]";
  }
  return stream.str();
}

inline std::string DescribeRosTime(const ros::Time& time) {
  std::ostringstream stream;
  stream << time.sec << "." << time.nsec;
  return stream.str();
}

inline std::string MissingTopicMessage(const std::string& input_bag_path,
                                      const std::string& topic_name,
                                      const std::string& expected_datatype,
                                      const std::vector<const rosbag::ConnectionInfo*>& connections) {
  std::ostringstream stream;
  stream << "  cause: requested topic/type not found"
         << "\n  input: " << input_bag_path
         << "\n  requested: " << topic_name << " [" << expected_datatype << "]"
         << "\n  available_topics:" << DescribeBagConnections(connections);
  return stream.str();
}

inline std::string MessageConversionFailure(const std::string& cause,
                                           const std::string& input_bag_path,
                                           const std::string& topic_name,
                                           const std::string& datatype,
                                           const ros::Time& time,
                                           const std::string& details) {
  std::ostringstream stream;
  stream << "  cause: " << cause
         << "\n  input: " << input_bag_path
         << "\n  message: " << topic_name << " [" << datatype << "]"
         << "\n  bag_time: " << DescribeRosTime(time)
         << "\n  details:\n" << details;
  return stream.str();
}

}  // namespace livox_lidar_rosbag_converter

#endif  // BAG_DIAGNOSTICS_HPP
