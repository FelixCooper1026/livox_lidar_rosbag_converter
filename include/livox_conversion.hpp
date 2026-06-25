#ifndef LIVOX_CONVERSION_HPP
#define LIVOX_CONVERSION_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace livox_lidar_rosbag_converter {

#pragma pack(push, 1)
struct LivoxPointXyzrtlt {
  float x;
  float y;
  float z;
  float intensity;
  uint8_t tag;
  uint8_t line;
  double timestamp;
};
#pragma pack(pop)

static_assert(sizeof(LivoxPointXyzrtlt) == 26, "Livox PointXYZRTLT layout must match livox_ros_driver2");

inline uint64_t StampToNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<uint64_t>(stamp.sec) * 1000000000ULL + static_cast<uint64_t>(stamp.nanosec);
}

inline std::vector<sensor_msgs::msg::PointField> MakeLivoxPointFields()
{
  std::vector<sensor_msgs::msg::PointField> fields(7);

  fields[0].name = "x";
  fields[0].offset = 0;
  fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[0].count = 1;

  fields[1].name = "y";
  fields[1].offset = 4;
  fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[1].count = 1;

  fields[2].name = "z";
  fields[2].offset = 8;
  fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[2].count = 1;

  fields[3].name = "intensity";
  fields[3].offset = 12;
  fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[3].count = 1;

  fields[4].name = "tag";
  fields[4].offset = 16;
  fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
  fields[4].count = 1;

  fields[5].name = "line";
  fields[5].offset = 17;
  fields[5].datatype = sensor_msgs::msg::PointField::UINT8;
  fields[5].count = 1;

  fields[6].name = "timestamp";
  fields[6].offset = 18;
  fields[6].datatype = sensor_msgs::msg::PointField::FLOAT64;
  fields[6].count = 1;

  return fields;
}

inline std::string PointFieldDatatypeName(uint8_t datatype)
{
  switch (datatype) {
    case sensor_msgs::msg::PointField::INT8:
      return "INT8";
    case sensor_msgs::msg::PointField::UINT8:
      return "UINT8";
    case sensor_msgs::msg::PointField::INT16:
      return "INT16";
    case sensor_msgs::msg::PointField::UINT16:
      return "UINT16";
    case sensor_msgs::msg::PointField::INT32:
      return "INT32";
    case sensor_msgs::msg::PointField::UINT32:
      return "UINT32";
    case sensor_msgs::msg::PointField::FLOAT32:
      return "FLOAT32";
    case sensor_msgs::msg::PointField::FLOAT64:
      return "FLOAT64";
    default:
      return "UNKNOWN(" + std::to_string(datatype) + ")";
  }
}

inline std::string DescribePointField(const sensor_msgs::msg::PointField & field)
{
  std::ostringstream stream;
  stream << field.name
         << "{offset=" << field.offset
         << ", datatype=" << PointFieldDatatypeName(field.datatype)
         << ", count=" << field.count
         << "}";
  return stream.str();
}

inline std::string DescribePointFieldsMultiline(
  const std::vector<sensor_msgs::msg::PointField> & fields, const std::string & indent)
{
  std::ostringstream stream;
  if (fields.empty()) {
    stream << indent << "- none";
    return stream.str();
  }

  for (const sensor_msgs::msg::PointField & field : fields) {
    stream << indent << "- " << DescribePointField(field) << "\n";
  }
  return stream.str();
}

inline std::string DescribeExpectedLivoxPointCloud2Format()
{
  std::ostringstream stream;
  stream << "Livox PointXYZRTLT (livox_ros_driver2 xfer_format=0)"
         << "\n    expected_height: 1"
         << "\n    expected_point_step: " << sizeof(LivoxPointXyzrtlt)
         << "\n    expected_fields:\n"
         << DescribePointFieldsMultiline(MakeLivoxPointFields(), "      ");
  return stream.str();
}

inline std::string DescribePointCloud2Format(const sensor_msgs::msg::PointCloud2 & msg)
{
  const size_t point_count = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
  std::ostringstream stream;
  stream << "PointCloud2"
         << "\n    frame_id: " << msg.header.frame_id
         << "\n    stamp_ns: " << StampToNanoseconds(msg.header.stamp)
         << "\n    height: " << msg.height
         << "\n    width: " << msg.width
         << "\n    points: " << point_count
         << "\n    point_step: " << msg.point_step
         << "\n    row_step: " << msg.row_step
         << "\n    is_bigendian: " << (msg.is_bigendian ? "true" : "false")
         << "\n    is_dense: " << (msg.is_dense ? "true" : "false")
         << "\n    data_size: " << msg.data.size()
         << "\n    fields:\n"
         << DescribePointFieldsMultiline(msg.fields, "      ");
  return stream.str();
}

inline std::string DescribePointFieldDifferences(const sensor_msgs::msg::PointCloud2 & msg)
{
  const std::vector<sensor_msgs::msg::PointField> expected_fields = MakeLivoxPointFields();
  const size_t shared_count = std::min(expected_fields.size(), msg.fields.size());
  std::ostringstream stream;

  if (msg.point_step != sizeof(LivoxPointXyzrtlt)) {
    stream << "      - point_step: expected " << sizeof(LivoxPointXyzrtlt)
           << ", actual " << msg.point_step << "\n";
  }

  const uint32_t expected_row_step = msg.width * sizeof(LivoxPointXyzrtlt);
  if (msg.row_step != expected_row_step) {
    stream << "      - row_step: expected " << expected_row_step
           << ", actual " << msg.row_step << "\n";
  }

  for (size_t i = 0; i < shared_count; ++i) {
    if (msg.fields[i].name != expected_fields[i].name ||
      msg.fields[i].offset != expected_fields[i].offset ||
      msg.fields[i].datatype != expected_fields[i].datatype ||
      msg.fields[i].count != expected_fields[i].count)
    {
      stream << "      - field[" << i << "]: expected "
             << DescribePointField(expected_fields[i])
             << ", actual " << DescribePointField(msg.fields[i]) << "\n";
    }
  }

  for (size_t i = shared_count; i < expected_fields.size(); ++i) {
    stream << "      - missing field[" << i << "]: "
           << DescribePointField(expected_fields[i]) << "\n";
  }

  for (size_t i = shared_count; i < msg.fields.size(); ++i) {
    stream << "      - unexpected field[" << i << "]: "
           << DescribePointField(msg.fields[i]) << "\n";
  }

  const size_t point_count = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
  const size_t expected_data_size = point_count * sizeof(LivoxPointXyzrtlt);
  if (msg.data.size() < expected_data_size) {
    stream << "      - data_size: expected at least " << expected_data_size
           << ", actual " << msg.data.size() << "\n";
  }

  const std::string differences = stream.str();
  return differences.empty() ? "      - none\n" : differences;
}

inline std::string FormatMismatchMessage(const std::string & summary, const sensor_msgs::msg::PointCloud2 & msg)
{
  std::ostringstream stream;
  stream << "    summary: " << summary
         << "\n    field_differences:\n" << DescribePointFieldDifferences(msg)
         << "    expected_format:\n    " << DescribeExpectedLivoxPointCloud2Format()
         << "    actual_format:\n    " << DescribePointCloud2Format(msg);
  return stream.str();
}

inline std::string DescribeCustomMsg(const livox_ros_driver2::msg::CustomMsg & msg)
{
  std::ostringstream stream;
  stream << "CustomMsg: frame_id=" << msg.header.frame_id
         << ", stamp_ns=" << StampToNanoseconds(msg.header.stamp)
         << ", timebase=" << msg.timebase
         << ", point_num=" << msg.point_num
         << ", points_size=" << msg.points.size()
         << ", lidar_id=" << static_cast<uint32_t>(msg.lidar_id);
  return stream.str();
}

inline void RequireLivoxPointCloud2Format(const sensor_msgs::msg::PointCloud2 & msg)
{
  const std::vector<sensor_msgs::msg::PointField> fields = MakeLivoxPointFields();
  if (msg.point_step != sizeof(LivoxPointXyzrtlt) || msg.fields.size() != fields.size()) {
    throw std::runtime_error(FormatMismatchMessage("PointCloud2 is not Livox PointXYZRTLT format", msg));
  }

  for (size_t i = 0; i < fields.size(); ++i) {
    if (msg.fields[i].name != fields[i].name ||
      msg.fields[i].offset != fields[i].offset ||
      msg.fields[i].datatype != fields[i].datatype ||
      msg.fields[i].count != fields[i].count)
    {
      throw std::runtime_error(FormatMismatchMessage("PointCloud2 field layout does not match Livox PointXYZRTLT", msg));
    }
  }

  const size_t point_count = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
  if (msg.data.size() < point_count * sizeof(LivoxPointXyzrtlt)) {
    throw std::runtime_error(FormatMismatchMessage("PointCloud2 data is shorter than Livox PointXYZRTLT point count", msg));
  }
}

inline sensor_msgs::msg::PointCloud2 ToPointCloud2(const livox_ros_driver2::msg::CustomMsg & msg)
{
  if (static_cast<size_t>(msg.point_num) != msg.points.size()) {
    throw std::runtime_error("CustomMsg point_num does not match points array size\n  actual: " + DescribeCustomMsg(msg));
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header = msg.header;
  cloud_msg.height = 1;
  cloud_msg.width = static_cast<uint32_t>(msg.points.size());
  cloud_msg.fields = MakeLivoxPointFields();
  cloud_msg.is_bigendian = false;
  cloud_msg.is_dense = true;
  cloud_msg.point_step = sizeof(LivoxPointXyzrtlt);
  cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step;
  cloud_msg.data.resize(cloud_msg.row_step);

  for (size_t i = 0; i < msg.points.size(); ++i) {
    const auto & src = msg.points[i];
    LivoxPointXyzrtlt dst;
    dst.x = src.x;
    dst.y = src.y;
    dst.z = src.z;
    dst.intensity = static_cast<float>(src.reflectivity);
    dst.tag = src.tag;
    dst.line = src.line;
    dst.timestamp = static_cast<double>(msg.timebase + src.offset_time);
    std::memcpy(&cloud_msg.data[i * sizeof(LivoxPointXyzrtlt)], &dst, sizeof(dst));
  }

  return cloud_msg;
}

inline livox_ros_driver2::msg::CustomMsg ToCustomMsg(const sensor_msgs::msg::PointCloud2 & msg)
{
  RequireLivoxPointCloud2Format(msg);

  const size_t point_count = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
  const uint64_t timebase = StampToNanoseconds(msg.header.stamp);

  livox_ros_driver2::msg::CustomMsg custom_msg;
  custom_msg.header = msg.header;
  custom_msg.timebase = timebase;
  custom_msg.point_num = static_cast<uint32_t>(point_count);
  custom_msg.lidar_id = 0;
  custom_msg.rsvd[0] = 0;
  custom_msg.rsvd[1] = 0;
  custom_msg.rsvd[2] = 0;
  custom_msg.points.resize(point_count);

  for (size_t i = 0; i < point_count; ++i) {
    LivoxPointXyzrtlt src;
    std::memcpy(&src, &msg.data[i * sizeof(LivoxPointXyzrtlt)], sizeof(src));

    auto & dst = custom_msg.points[i];
    dst.x = src.x;
    dst.y = src.y;
    dst.z = src.z;
    dst.reflectivity = static_cast<uint8_t>(src.intensity);
    dst.tag = src.tag;
    dst.line = src.line;
    dst.offset_time = static_cast<uint32_t>(std::llround(src.timestamp - static_cast<double>(timebase)));
  }

  return custom_msg;
}

}  // namespace livox_lidar_rosbag_converter

#endif  // LIVOX_CONVERSION_HPP
