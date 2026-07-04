#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class OccupancyProjectionNode : public rclcpp::Node
{
public:
  OccupancyProjectionNode() : Node("occupancy_projection_node")
  {
    input_cloud_topic_ = this->declare_parameter<std::string>("input_cloud_topic", "/lidar/points_filtered");
    input_odom_topic_ = this->declare_parameter<std::string>("input_odom_topic", "/nav/odom");
    output_grid_topic_ = this->declare_parameter<std::string>("output_grid_topic", "/planning/occupancy_updates");
    output_marker_topic_ = this->declare_parameter<std::string>("output_marker_topic", "/planning/occupancy_markers");

    grid_width_m_ = this->declare_parameter<double>("grid_width_m", 30.0);
    grid_height_m_ = this->declare_parameter<double>("grid_height_m", 30.0);
    grid_resolution_ = this->declare_parameter<double>("grid_resolution", 0.5);
    grid_origin_x_ = this->declare_parameter<double>("grid_origin_x", -10.0);
    grid_origin_y_ = this->declare_parameter<double>("grid_origin_y", -10.0);

    slice_mode_ = this->declare_parameter<std::string>("slice_mode", "relative_to_vehicle");
    fly_altitude_ = this->declare_parameter<double>("fly_altitude", 2.0);
    slice_z_min_rel_ = this->declare_parameter<double>("slice_z_min_rel", -1.0);
    slice_z_max_rel_ = this->declare_parameter<double>("slice_z_max_rel", 1.0);
    min_points_per_cell_ = this->declare_parameter<int>("min_points_per_cell", 2);
    marker_height_ = this->declare_parameter<double>("marker_height", 2.0);

    grid_w_ = static_cast<int>(std::round(grid_width_m_ / grid_resolution_));
    grid_h_ = static_cast<int>(std::round(grid_height_m_ / grid_resolution_));

    if (grid_w_ <= 0 || grid_h_ <= 0) {
      throw std::runtime_error("grid size must be positive");
    }

    grid_counts_.assign(static_cast<size_t>(grid_w_ * grid_h_), 0);
    grid_msg_.data.assign(static_cast<size_t>(grid_w_ * grid_h_), 0);

    grid_msg_.header.frame_id = "world";
    grid_msg_.info.resolution = grid_resolution_;
    grid_msg_.info.width = static_cast<uint32_t>(grid_w_);
    grid_msg_.info.height = static_cast<uint32_t>(grid_h_);
    grid_msg_.info.origin.position.x = grid_origin_x_;
    grid_msg_.info.origin.position.y = grid_origin_y_;
    grid_msg_.info.origin.position.z = 0.0;
    grid_msg_.info.origin.orientation.w = 1.0;

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_odom_topic_,
      10,
      std::bind(&OccupancyProjectionNode::odomCallback, this, std::placeholders::_1));

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&OccupancyProjectionNode::cloudCallback, this, std::placeholders::_1));

    grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(output_grid_topic_, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(output_marker_topic_, 1);

    RCLCPP_INFO(this->get_logger(),
      "[Occupancy Projection] cloud=%s odom=%s grid=%dx%d res=%.2f slice_mode=%s",
      input_cloud_topic_.c_str(), input_odom_topic_.c_str(),
      grid_w_, grid_h_, grid_resolution_, slice_mode_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    cur_z_ = msg->pose.pose.position.z;
    odom_received_ = true;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (slice_mode_ == "relative_to_vehicle" && !odom_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[Occupancy Projection] odom 아직 못 받음. cloud 무시함");
      return;
    }

    const double z_ref = (slice_mode_ == "relative_to_vehicle") ? cur_z_ : fly_altitude_;
    const double z_min = z_ref + slice_z_min_rel_;
    const double z_max = z_ref + slice_z_max_rel_;

    std::fill(grid_counts_.begin(), grid_counts_.end(), 0);
    std::fill(grid_msg_.data.begin(), grid_msg_.data.end(), 0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    int in_slice_points = 0;   // z-slice 통과한 포인트 수 기록용 
    int in_grid_points = 0;    // grid 범위 안에 들어온 포인트 수 기록용
    bool first_point_logged = false;  // 첫 포인트 한 번만 로그 찍기용

    for (const auto & pt : cloud->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }

      if (!first_point_logged) {
        RCLCPP_INFO(this->get_logger(),
          "[Occupancy Projection Debug] first_pt=(%.2f, %.2f, %.2f)",
          pt.x, pt.y, pt.z);
        first_point_logged = true;
      }

      if (pt.z < z_min || pt.z > z_max) {
        continue;
      }

      in_slice_points++;

      const int xi = static_cast<int>(std::floor((pt.x - grid_origin_x_) / grid_resolution_));
      const int yi = static_cast<int>(std::floor((pt.y - grid_origin_y_) / grid_resolution_));

      if (!inBounds(xi, yi)) {
        continue;
      }

      in_grid_points++;

      const size_t idx = toIndex(xi, yi);
      grid_counts_[idx] += 1;
    }

    int occupied_count = 0;
    for (size_t i = 0; i < grid_counts_.size(); ++i) {
      if (grid_counts_[i] >= min_points_per_cell_) {
        grid_msg_.data[i] = 100;
        occupied_count++;
      }
    }

    grid_msg_.header.stamp = msg->header.stamp;
    grid_pub_->publish(grid_msg_);
    publishMarkers(msg->header.stamp);

    RCLCPP_INFO(this->get_logger(),
      "[Occupancy Projection Debug] cloud_pts=%zu z_ref=%.2f z_slice=[%.2f, %.2f] in_slice=%d in_grid=%d occupied_cells=%d min_points_per_cell=%d",
      cloud->size(), z_ref, z_min, z_max, in_slice_points, in_grid_points, occupied_count, min_points_per_cell_);
  }

  void publishMarkers(const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray ma;

    visualization_msgs::msg::Marker del;
    del.header.frame_id = "world";
    del.header.stamp = stamp;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);

    visualization_msgs::msg::Marker cubes;
    cubes.header.frame_id = "world";
    cubes.header.stamp = stamp;
    cubes.ns = "occupancy_projection";
    cubes.id = 0;
    cubes.type = visualization_msgs::msg::Marker::CUBE_LIST;
    cubes.action = visualization_msgs::msg::Marker::ADD;
    cubes.pose.orientation.w = 1.0;
    cubes.scale.x = grid_resolution_;
    cubes.scale.y = grid_resolution_;
    cubes.scale.z = marker_height_;
    cubes.color.r = 1.0f;
    cubes.color.g = 0.3f;
    cubes.color.b = 0.1f;
    cubes.color.a = 0.55f;

    for (int yi = 0; yi < grid_h_; ++yi) {
      for (int xi = 0; xi < grid_w_; ++xi) {
        const size_t idx = toIndex(xi, yi);
        if (grid_msg_.data[idx] <= 0) {
          continue;
        }

        geometry_msgs::msg::Point p;
        p.x = grid_origin_x_ + (static_cast<double>(xi) + 0.5) * grid_resolution_;
        p.y = grid_origin_y_ + (static_cast<double>(yi) + 0.5) * grid_resolution_;
        p.z = marker_height_ * 0.5;
        cubes.points.push_back(p);
      }
    }

    ma.markers.push_back(cubes);
    marker_pub_->publish(ma);
  }

  bool inBounds(int xi, int yi) const
  {
    return xi >= 0 && xi < grid_w_ && yi >= 0 && yi < grid_h_;
  }

  size_t toIndex(int xi, int yi) const
  {
    return static_cast<size_t>(yi * grid_w_ + xi);
  }

  std::string input_cloud_topic_;
  std::string input_odom_topic_;
  std::string output_grid_topic_;
  std::string output_marker_topic_;
  std::string slice_mode_;

  double grid_width_m_{30.0};
  double grid_height_m_{30.0};
  double grid_resolution_{0.5};
  double grid_origin_x_{-10.0};
  double grid_origin_y_{-10.0};
  double fly_altitude_{2.0};
  double slice_z_min_rel_{-1.0};
  double slice_z_max_rel_{1.0};
  double marker_height_{2.0};
  int min_points_per_cell_{2};

  int grid_w_{0};
  int grid_h_{0};
  double cur_z_{0.0};
  bool odom_received_{false};

  std::vector<int> grid_counts_;
  nav_msgs::msg::OccupancyGrid grid_msg_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyProjectionNode>());
  rclcpp::shutdown();
  return 0;
}
