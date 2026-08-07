#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>

class Px4TrackingRmseNode : public rclcpp::Node
{
public:
  Px4TrackingRmseNode() : Node("px4_tracking_rmse_node")
  {
    case_name_ = declare_parameter<std::string>("case_name", "px4_eval");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/nav/odom");
    setpoint_topic_ = declare_parameter<std::string>("setpoint_topic", "/guidance/setpoint");
    output_dir_ = declare_parameter<std::string>("output_dir", "/home/lyj/uav_gnc_ws/eval/px4_lio_compare");
    sample_rate_hz_ = declare_parameter<double>("sample_rate_hz", 20.0);
    min_eval_time_sec_ = declare_parameter<double>("min_eval_time_sec", 3.0);
    max_setpoint_age_sec_ = declare_parameter<double>("max_setpoint_age_sec", 0.5);

    std::filesystem::create_directories(output_dir_);
    sample_csv_path_ = output_dir_ + "/" + case_name_ + "_samples.csv";
    summary_csv_path_ = output_dir_ + "/" + case_name_ + "_summary.csv";

    sample_csv_.open(sample_csv_path_, std::ios::out);
    if (!sample_csv_.is_open()) {
      RCLCPP_FATAL(get_logger(), "Failed to open sample csv: %s", sample_csv_path_.c_str());
      throw std::runtime_error("sample csv open failed");
    }

    sample_csv_
      << "t_sec,"
      << "x,y,z,vx,vy,vz,"
      << "ref_x,ref_y,ref_z,ref_vx,ref_vy,ref_vz,"
      << "err_x,err_y,err_z,err_xy,err_z_abs,err_3d,"
      << "speed,ref_speed\n";
    sample_csv_.flush();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Px4TrackingRmseNode::odomCallback, this, std::placeholders::_1));

    setpoint_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      setpoint_topic_, 20,
      std::bind(&Px4TrackingRmseNode::setpointCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, sample_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Px4TrackingRmseNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "[PX4 Tracking RMSE] case=%s odom=%s setpoint=%s out=%s",
      case_name_.c_str(), odom_topic_.c_str(), setpoint_topic_.c_str(), output_dir_.c_str());
  }

  ~Px4TrackingRmseNode() override
  {
    writeSummary();
    if (sample_csv_.is_open()) {
      sample_csv_.close();
    }
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = *msg;
    have_odom_ = true;
  }

  void setpointCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_setpoint_ = *msg;
    latest_setpoint_wall_time_ = now();
    have_setpoint_ = true;
  }

  void onTimer()
  {
    if (!have_odom_ || !have_setpoint_) {
      return;
    }

    const auto now_time = now();
    if ((now_time - latest_setpoint_wall_time_).seconds() > max_setpoint_age_sec_) {
      return;
    }

    if (!started_) {
      started_ = true;
      start_time_ = now_time;
    }

    const double t = (now_time - start_time_).seconds();

    const auto &p = latest_odom_.pose.pose.position;
    const auto &v = latest_odom_.twist.twist.linear;
    const auto &rp = latest_setpoint_.pose.pose.position;
    const auto &rv = latest_setpoint_.twist.twist.linear;

    const double ex = p.x - rp.x;
    const double ey = p.y - rp.y;
    const double ez = p.z - rp.z;
    const double err_xy = std::hypot(ex, ey);
    const double err_z_abs = std::abs(ez);
    const double err_3d = std::sqrt(ex * ex + ey * ey + ez * ez);
    const double speed = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    const double ref_speed = std::sqrt(rv.x * rv.x + rv.y * rv.y + rv.z * rv.z);

    sample_csv_ << std::fixed << std::setprecision(6)
      << t << ","
      << p.x << "," << p.y << "," << p.z << ","
      << v.x << "," << v.y << "," << v.z << ","
      << rp.x << "," << rp.y << "," << rp.z << ","
      << rv.x << "," << rv.y << "," << rv.z << ","
      << ex << "," << ey << "," << ez << ","
      << err_xy << "," << err_z_abs << "," << err_3d << ","
      << speed << "," << ref_speed << "\n";

    if ((samples_ % 50U) == 0U) {
      sample_csv_.flush();
    }

    samples_++;
    sum_sq_xy_ += err_xy * err_xy;
    sum_sq_z_ += ez * ez;
    sum_sq_3d_ += err_3d * err_3d;
    sum_abs_xy_ += err_xy;
    sum_abs_z_ += err_z_abs;
    sum_abs_3d_ += err_3d;
    max_xy_ = std::max(max_xy_, err_xy);
    max_z_ = std::max(max_z_, err_z_abs);
    max_3d_ = std::max(max_3d_, err_3d);
    final_xy_ = err_xy;
    final_z_ = err_z_abs;
    final_3d_ = err_3d;
    duration_sec_ = t;
  }

  void writeSummary()
  {
    if (summary_written_) {
      return;
    }
    summary_written_ = true;

    std::ofstream summary(summary_csv_path_, std::ios::out);
    if (!summary.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open summary csv: %s", summary_csv_path_.c_str());
      return;
    }

    summary
      << "case_name,samples,duration_sec,valid_eval,"
      << "rmse_xy,rmse_z,rmse_3d,"
      << "mae_xy,mae_z,mae_3d,"
      << "max_xy,max_z,max_3d,"
      << "final_xy,final_z,final_3d\n";

    const bool valid = samples_ > 0U && duration_sec_ >= min_eval_time_sec_;
    const double n = static_cast<double>(std::max<size_t>(samples_, 1U));

    summary << std::fixed << std::setprecision(6)
      << case_name_ << ","
      << samples_ << ","
      << duration_sec_ << ","
      << (valid ? 1 : 0) << ","
      << std::sqrt(sum_sq_xy_ / n) << ","
      << std::sqrt(sum_sq_z_ / n) << ","
      << std::sqrt(sum_sq_3d_ / n) << ","
      << sum_abs_xy_ / n << ","
      << sum_abs_z_ / n << ","
      << sum_abs_3d_ / n << ","
      << max_xy_ << ","
      << max_z_ << ","
      << max_3d_ << ","
      << final_xy_ << ","
      << final_z_ << ","
      << final_3d_ << "\n";
    summary.close();

    RCLCPP_INFO(
      get_logger(),
      "[PX4 Tracking RMSE] case=%s samples=%lu duration=%.1fs rmse_xy=%.3fm rmse_z=%.3fm rmse_3d=%.3fm summary=%s",
      case_name_.c_str(), samples_, duration_sec_,
      std::sqrt(sum_sq_xy_ / n), std::sqrt(sum_sq_z_ / n), std::sqrt(sum_sq_3d_ / n),
      summary_csv_path_.c_str());
  }

  std::string case_name_;
  std::string odom_topic_;
  std::string setpoint_topic_;
  std::string output_dir_;
  std::string sample_csv_path_;
  std::string summary_csv_path_;
  double sample_rate_hz_{20.0};
  double min_eval_time_sec_{3.0};
  double max_setpoint_age_sec_{0.5};

  bool have_odom_{false};
  bool have_setpoint_{false};
  bool started_{false};
  bool summary_written_{false};
  nav_msgs::msg::Odometry latest_odom_;
  nav_msgs::msg::Odometry latest_setpoint_;
  rclcpp::Time latest_setpoint_wall_time_;
  rclcpp::Time start_time_;

  size_t samples_{0};
  double duration_sec_{0.0};
  double sum_sq_xy_{0.0};
  double sum_sq_z_{0.0};
  double sum_sq_3d_{0.0};
  double sum_abs_xy_{0.0};
  double sum_abs_z_{0.0};
  double sum_abs_3d_{0.0};
  double max_xy_{0.0};
  double max_z_{0.0};
  double max_3d_{0.0};
  double final_xy_{std::numeric_limits<double>::quiet_NaN()};
  double final_z_{std::numeric_limits<double>::quiet_NaN()};
  double final_3d_{std::numeric_limits<double>::quiet_NaN()};

  std::ofstream sample_csv_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr setpoint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4TrackingRmseNode>());
  rclcpp::shutdown();
  return 0;
}
