#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "uav_dynamics/sixdof.hpp"

using namespace std::chrono_literals;

class RawLidarSimNode : public rclcpp::Node
{
public:
  RawLidarSimNode() : Node("raw_lidar_sim_node")
  {
    truth_odom_topic_ = declare_parameter<std::string>("truth_odom_topic", "/sim/odom");
    output_topic_ = declare_parameter<std::string>("output_topic", "/lidar/points_raw");
    frame_id_ = declare_parameter<std::string>("frame_id", "lidar");

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    range_min_ = declare_parameter<double>("range_min_m", 0.5);
    range_max_ = declare_parameter<double>("range_max_m", 15.0);
    h_fov_deg_ = declare_parameter<double>("horizontal_fov_deg", 360.0);
    v_fov_deg_ = declare_parameter<double>("vertical_fov_deg", 30.0);
    h_samples_ = declare_parameter<int>("horizontal_samples", 360);
    v_samples_ = declare_parameter<int>("vertical_samples", 16);
    noise_std_m_ = declare_parameter<double>("range_noise_std_m", 0.01);

    lidar_xyz_body_.x = declare_parameter<double>("lidar_x_body_m", 0.0);
    lidar_xyz_body_.y = declare_parameter<double>("lidar_y_body_m", 0.0);
    lidar_xyz_body_.z = declare_parameter<double>("lidar_z_body_m", 0.05);
    lidar_rpy_body_.x = declare_parameter<double>("lidar_roll_body_rad", 0.0);
    lidar_rpy_body_.y = declare_parameter<double>("lidar_pitch_body_rad", 0.0);
    lidar_rpy_body_.z = declare_parameter<double>("lidar_yaw_body_rad", 0.0);
    q_body_lidar_ = rpyToQuat(lidar_rpy_body_.x, lidar_rpy_body_.y, lidar_rpy_body_.z);

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

    rng_.seed(static_cast<uint32_t>(declare_parameter<int>("random_seed", 11)));
    noise_ = std::normal_distribution<double>(0.0, std::max(0.0, noise_std_m_));

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::SensorDataQoS());
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      truth_odom_topic_, 10,
      std::bind(&RawLidarSimNode::odomCallback, this, std::placeholders::_1));

    const int period_ms = static_cast<int>(1000.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms), std::bind(&RawLidarSimNode::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "[Raw LiDAR Sim] truth=%s output=%s frame=%s obs=%zu samples=%dx%d range=[%.1f, %.1f]",
      truth_odom_topic_.c_str(), output_topic_.c_str(), frame_id_.c_str(),
      obstacles_.size(), h_samples_, v_samples_, range_min_, range_max_);
  }

private:
  struct Obstacle
  {
    double x;
    double y;
    double r;
  };

  struct Hit
  {
    Vec3 p_lidar;
    float time_offset;
    float intensity;
    uint16_t ring;
  };

  static Quat rpyToQuat(double roll, double pitch, double yaw)
  {
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    Quat q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    q.normalize();
    return q;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    base_pos_.x = msg->pose.pose.position.x;
    base_pos_.y = msg->pose.pose.position.y;
    base_pos_.z = msg->pose.pose.position.z;

    q_world_body_.w = msg->pose.pose.orientation.w;
    q_world_body_.x = msg->pose.pose.orientation.x;
    q_world_body_.y = msg->pose.pose.orientation.y;
    q_world_body_.z = msg->pose.pose.orientation.z;
    q_world_body_.normalize();
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
    double best_t = std::numeric_limits<double>::infinity();
    for (double t : {(-b - sqrt_disc) / (2.0 * a), (-b + sqrt_disc) / (2.0 * a)}) {
      if (t < range_min_ || t > range_max_) continue;
      const double z_hit = p0.z + t * dir.z;
      if (z_hit < obstacle_base_z_ || z_hit > obstacle_base_z_ + obstacle_height_) continue;
      best_t = std::min(best_t, t);
    }

    if (!std::isfinite(best_t)) return false;
    t_hit = best_t;
    return true;
  }

  void onTimer()
  {
    if (!odom_received_) return;

    const Vec3 lidar_origin_world = base_pos_ + q_world_body_.rotateBodyToWorld(lidar_xyz_body_);
    const Quat q_world_lidar = Quat::multiply(q_world_body_, q_body_lidar_);

    std::vector<Hit> hits;
    hits.reserve(static_cast<size_t>(h_samples_ * v_samples_));

    const double h_fov_rad = h_fov_deg_ * M_PI / 180.0;
    const double v_fov_rad = v_fov_deg_ * M_PI / 180.0;
    const int total_rays = std::max(1, h_samples_ * v_samples_);
    int ray_index = 0;

    for (int vi = 0; vi < v_samples_; ++vi) {
      const double elev = (v_samples_ == 1)
        ? 0.0
        : (-0.5 * v_fov_rad + v_fov_rad * static_cast<double>(vi) / static_cast<double>(v_samples_ - 1));
      for (int hi = 0; hi < h_samples_; ++hi, ++ray_index) {
        const double azim = (h_samples_ == 1)
          ? 0.0
          : (-0.5 * h_fov_rad + h_fov_rad * static_cast<double>(hi) / static_cast<double>(h_samples_ - 1));

        Vec3 dir_lidar(
          std::cos(elev) * std::cos(azim),
          std::cos(elev) * std::sin(azim),
          std::sin(elev));
        Vec3 dir_world = q_world_lidar.rotateBodyToWorld(dir_lidar);
        const double n = dir_world.norm();
        if (n < 1e-10) continue;
        dir_world = dir_world / n;

        double best_t = std::numeric_limits<double>::infinity();
        bool found = false;
        for (const auto &obs : obstacles_) {
          double t_hit = 0.0;
          if (!intersectRayCylinder(lidar_origin_world, dir_world, obs, t_hit)) continue;
          if (t_hit < best_t) {
            best_t = t_hit;
            found = true;
          }
        }
        if (!found) continue;

        const double noisy_t = std::clamp(best_t + noise_(rng_), range_min_, range_max_);
        hits.push_back({
          dir_lidar * noisy_t,
          static_cast<float>(static_cast<double>(ray_index) / total_rays / publish_rate_hz_),
          1.0f,
          static_cast<uint16_t>(vi)});
      }
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = now();
    cloud_msg.header.frame_id = frame_id_;
    cloud_msg.height = 1;
    cloud_msg.width = static_cast<uint32_t>(hits.size());
    cloud_msg.is_bigendian = false;
    cloud_msg.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2Fields(
      6,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "time", 1, sensor_msgs::msg::PointField::FLOAT32,
      "ring", 1, sensor_msgs::msg::PointField::UINT16);
    modifier.resize(hits.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_msg, "intensity");
    sensor_msgs::PointCloud2Iterator<float> iter_time(cloud_msg, "time");
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring(cloud_msg, "ring");
    for (const auto &hit : hits) {
      *iter_x = static_cast<float>(hit.p_lidar.x);
      *iter_y = static_cast<float>(hit.p_lidar.y);
      *iter_z = static_cast<float>(hit.p_lidar.z);
      *iter_intensity = hit.intensity;
      *iter_time = hit.time_offset;
      *iter_ring = hit.ring;
      ++iter_x; ++iter_y; ++iter_z; ++iter_intensity; ++iter_time; ++iter_ring;
    }

    cloud_pub_->publish(cloud_msg);
  }

  std::string truth_odom_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double publish_rate_hz_{10.0};
  double range_min_{0.5};
  double range_max_{15.0};
  double h_fov_deg_{360.0};
  double v_fov_deg_{30.0};
  int h_samples_{360};
  int v_samples_{16};
  double noise_std_m_{0.01};
  double obstacle_base_z_{0.0};
  double obstacle_height_{3.0};

  Vec3 base_pos_;
  Quat q_world_body_;
  Vec3 lidar_xyz_body_;
  Vec3 lidar_rpy_body_;
  Quat q_body_lidar_;
  bool odom_received_{false};

  std::vector<Obstacle> obstacles_;
  std::mt19937 rng_;
  std::normal_distribution<double> noise_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RawLidarSimNode>());
  rclcpp::shutdown();
  return 0;
}
