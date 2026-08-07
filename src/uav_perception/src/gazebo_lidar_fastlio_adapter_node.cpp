#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class GazeboLidarFastlioAdapterNode : public rclcpp::Node
{
public:
  GazeboLidarFastlioAdapterNode() : Node("gazebo_lidar_fastlio_adapter_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/gazebo/lidar/points_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/lidar/points_raw");
    frame_id_ = declare_parameter<std::string>("frame_id", "lidar");
    scan_line_ = declare_parameter<int>("scan_line", 32);
    scan_rate_hz_ = declare_parameter<double>("scan_rate_hz", 20.0);
    restamp_with_ros_time_ = declare_parameter<bool>("restamp_with_ros_time", true);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GazeboLidarFastlioAdapterNode::cloudCallback, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(),
      "[Gazebo LiDAR FAST-LIO Adapter] input=%s output=%s frame=%s scan_line=%d scan_rate=%.1fHz restamp=%s",
      input_topic_.c_str(), output_topic_.c_str(), frame_id_.c_str(), scan_line_, scan_rate_hz_,
      restamp_with_ros_time_ ? "true" : "false");
  }

private:
  static const sensor_msgs::msg::PointField * findField(
    const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
  {
    for (const auto & field : msg.fields) {
      if (field.name == name) {
        return &field;
      }
    }
    return nullptr;
  }

  static float readFloat32(
    const sensor_msgs::msg::PointCloud2 & msg,
    const sensor_msgs::msg::PointField * field,
    const uint32_t point_index,
    const float fallback)
  {
    if (field == nullptr || field->datatype != sensor_msgs::msg::PointField::FLOAT32) {
      return fallback;
    }
    const size_t offset = static_cast<size_t>(point_index) * msg.point_step + field->offset;
    if (offset + sizeof(float) > msg.data.size()) {
      return fallback;
    }
    float value = fallback;
    std::memcpy(&value, msg.data.data() + offset, sizeof(float));
    return value;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto * x_field = findField(*msg, "x");
    const auto * y_field = findField(*msg, "y");
    const auto * z_field = findField(*msg, "z");
    const auto * intensity_field = findField(*msg, "intensity");

    if (x_field == nullptr || y_field == nullptr || z_field == nullptr) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping Gazebo LiDAR cloud because x/y/z fields are missing.");
      return;
    }

    const uint32_t point_count = msg->width * msg->height;
    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    out.header.frame_id = frame_id_;
    if (restamp_with_ros_time_) {
      out.header.stamp = now();
    }
    out.height = 1;
    out.width = point_count;
    out.is_bigendian = false;
    out.is_dense = msg->is_dense;

    sensor_msgs::PointCloud2Modifier modifier(out);
    modifier.setPointCloud2Fields(
      6,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "time", 1, sensor_msgs::msg::PointField::FLOAT32,
      "ring", 1, sensor_msgs::msg::PointField::UINT16);
    modifier.resize(point_count);

    sensor_msgs::PointCloud2Iterator<float> out_x(out, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(out, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(out, "z");
    sensor_msgs::PointCloud2Iterator<float> out_intensity(out, "intensity");
    sensor_msgs::PointCloud2Iterator<float> out_time(out, "time");
    sensor_msgs::PointCloud2Iterator<uint16_t> out_ring(out, "ring");

    const double scan_period = scan_rate_hz_ > 1e-6 ? 1.0 / scan_rate_hz_ : 0.05;
    const int scan_line = std::max(scan_line_, 1);
    const uint32_t denom = point_count > 1 ? point_count - 1 : 1;

    for (uint32_t i = 0; i < point_count;
         ++i, ++out_x, ++out_y, ++out_z, ++out_intensity, ++out_time, ++out_ring) {
      *out_x = readFloat32(*msg, x_field, i, 0.0F);
      *out_y = readFloat32(*msg, y_field, i, 0.0F);
      *out_z = readFloat32(*msg, z_field, i, 0.0F);
      *out_intensity = readFloat32(*msg, intensity_field, i, 1.0F);
      *out_time = static_cast<float>((static_cast<double>(i) / denom) * scan_period);
      *out_ring = static_cast<uint16_t>(i % static_cast<uint32_t>(scan_line));
    }

    pub_->publish(out);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[Gazebo LiDAR FAST-LIO Adapter] converted points=%u stamp=%.3f",
      point_count,
      rclcpp::Time(out.header.stamp).seconds());
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  int scan_line_{32};
  double scan_rate_hz_{20.0};
  bool restamp_with_ros_time_{true};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboLidarFastlioAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
