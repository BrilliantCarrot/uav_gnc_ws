#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "uav_gnc/sixdof.h"

using namespace std::chrono_literals;

class VirtualLidarNode : public rclcpp::Node
{
public:
  VirtualLidarNode() : Node("virtual_lidar_node")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/sim/odom");
    output_topic_ = declare_parameter<std::string>("output_topic", "/lidar/points");
    frame_id_ = declare_parameter<std::string>("frame_id", "world");

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    range_min_ = declare_parameter<double>("range_min_m", 0.5);
    range_max_ = declare_parameter<double>("range_max_m", 15.0);

    h_fov_deg_ = declare_parameter<double>("horizontal_fov_deg", 360.0);
    v_fov_deg_ = declare_parameter<double>("vertical_fov_deg", 30.0);
    h_samples_ = declare_parameter<int>("horizontal_samples", 180);
    v_samples_ = declare_parameter<int>("vertical_samples", 8);

    obstacle_base_z_ = declare_parameter<double>("obstacle_base_z", 0.0);
    obstacle_height_ = declare_parameter<double>("obstacle_height", 3.0);

    auto obs_x = declare_parameter<std::vector<double>>("obstacle_x", std::vector<double>{});
    auto obs_y = declare_parameter<std::vector<double>>("obstacle_y", std::vector<double>{});
    auto obs_r = declare_parameter<std::vector<double>>("obstacle_r", std::vector<double>{});

    const size_t n = std::min({obs_x.size(), obs_y.size(), obs_r.size()});
    obstacles_.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      obstacles_.push_back({obs_x[i], obs_y[i], obs_r[i]});
    }

    if (obs_x.size() != obs_y.size() || obs_x.size() != obs_r.size()) {
      RCLCPP_WARN(get_logger(),
        "[Virtual LiDAR] obstacle_x/y/r 크기 다름. 최소 길이 %zu개만 사용함", n);
    }

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&VirtualLidarNode::odomCallback, this, std::placeholders::_1));

    const int period_ms = static_cast<int>(1000.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&VirtualLidarNode::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "[Virtual LiDAR] odom=%s output=%s obs=%zu h_samples=%d v_samples=%d range=[%.1f, %.1f]",
      odom_topic_.c_str(), output_topic_.c_str(), obstacles_.size(),
      h_samples_, v_samples_, range_min_, range_max_);
  }

private:
  struct Obstacle
  {
    double x;
    double y;
    double r;
  };

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    pos_.x = msg->pose.pose.position.x;
    pos_.y = msg->pose.pose.position.y;
    pos_.z = msg->pose.pose.position.z;

    q_.w = msg->pose.pose.orientation.w;
    q_.x = msg->pose.pose.orientation.x;
    q_.y = msg->pose.pose.orientation.y;
    q_.z = msg->pose.pose.orientation.z;
    q_.normalize();

    odom_received_ = true;
  }

  bool intersectRayCylinder(const Vec3 &p0, const Vec3 &dir, const Obstacle &obs, double &t_hit) const
  {
    const double a = dir.x * dir.x + dir.y * dir.y;
    if (a < 1e-10) return false;

    const double dx = p0.x - obs.x;
    const double dy = p0.y - obs.y;
    const double b = 2.0 * (dx * dir.x + dy * dir.y);
    const double c = dx * dx + dy * dy - obs.r * obs.r;
    const double disc = b * b - 4.0 * a * c;

    if (disc < 0.0) return false;

    const double sqrt_disc = std::sqrt(disc);
    const double t1 = (-b - sqrt_disc) / (2.0 * a);
    const double t2 = (-b + sqrt_disc) / (2.0 * a);

    double best_t = std::numeric_limits<double>::infinity();

    auto check_candidate = [&](double t) {
      if (t < range_min_ || t > range_max_) return;
      const double z_hit = p0.z + t * dir.z;
      if (z_hit < obstacle_base_z_ || z_hit > (obstacle_base_z_ + obstacle_height_)) return;
      if (t < best_t) best_t = t;
    };

    check_candidate(t1);
    check_candidate(t2);

    if (!std::isfinite(best_t)) return false;
    t_hit = best_t;
    return true;
  }

  void onTimer()
  {
    if (!odom_received_) return;

    std::vector<Vec3> hits;
    hits.reserve(static_cast<size_t>(h_samples_ * v_samples_));

    const double h_fov_rad = h_fov_deg_ * M_PI / 180.0;
    const double v_fov_rad = v_fov_deg_ * M_PI / 180.0;

    for (int vi = 0; vi < v_samples_; ++vi) {
      const double elev = (v_samples_ == 1)
        ? 0.0
        : (-0.5 * v_fov_rad + v_fov_rad * static_cast<double>(vi) / static_cast<double>(v_samples_ - 1));

      for (int hi = 0; hi < h_samples_; ++hi) {
        const double azim = (h_samples_ == 1)
          ? 0.0
          : (-0.5 * h_fov_rad + h_fov_rad * static_cast<double>(hi) / static_cast<double>(h_samples_ - 1));

        // body x축 전방 기준 beam 생성함
        Vec3 dir_body(
          std::cos(elev) * std::cos(azim),
          std::cos(elev) * std::sin(azim),
          std::sin(elev));

        // body -> world 회전 적용함
        Vec3 dir_world = q_.rotateBodyToWorld(dir_body);
        const double n = dir_world.norm();
        if (n < 1e-10) continue;
        dir_world = dir_world / n;

        double best_t = std::numeric_limits<double>::infinity();
        bool found = false;

        for (const auto &obs : obstacles_) {
          double t_hit = 0.0;
          if (!intersectRayCylinder(pos_, dir_world, obs, t_hit)) continue;
          if (t_hit < best_t) {
            best_t = t_hit;
            found = true;
          }
        }

        if (!found) continue;

        hits.emplace_back(
          pos_.x + best_t * dir_world.x,
          pos_.y + best_t * dir_world.y,
          pos_.z + best_t * dir_world.z);
      }
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = now();
    cloud_msg.header.frame_id = frame_id_;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(hits.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (const auto &p : hits) {
      *iter_x = static_cast<float>(p.x);
      *iter_y = static_cast<float>(p.y);
      *iter_z = static_cast<float>(p.z);
      ++iter_x; ++iter_y; ++iter_z;
    }

    cloud_pub_->publish(cloud_msg);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[Virtual LiDAR] published %zu points at pos=(%.2f, %.2f, %.2f)",
      hits.size(), pos_.x, pos_.y, pos_.z);
  }

  std::string odom_topic_;
  std::string output_topic_;
  std::string frame_id_;

  double publish_rate_hz_{10.0};
  double range_min_{0.5};
  double range_max_{15.0};
  double h_fov_deg_{360.0};
  double v_fov_deg_{30.0};
  int h_samples_{180};
  int v_samples_{8};

  double obstacle_base_z_{0.0};
  double obstacle_height_{3.0};

  bool odom_received_{false};
  Vec3 pos_;
  Quat q_;

  std::vector<Obstacle> obstacles_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VirtualLidarNode>());
  rclcpp::shutdown();
  return 0;
}