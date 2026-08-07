#include <cmath>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class LioValidationErrorNode : public rclcpp::Node
{
public:
  LioValidationErrorNode() : Node("lio_validation_error_node")
  {
    truth_topic_ = declare_parameter<std::string>("truth_topic", "/lio_validation/odom_truth");
    lio_topic_ = declare_parameter<std::string>("lio_topic", "/lio/odom");
    log_period_ms_ = declare_parameter<int>("log_period_ms", 1000);
    min_samples_ = declare_parameter<int>("min_samples", 5);
    compare_relative_motion_ = declare_parameter<bool>("compare_relative_motion", true);

    truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      truth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LioValidationErrorNode::truthCallback, this, std::placeholders::_1));
    lio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      lio_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LioValidationErrorNode::lioCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "[LIO Validation Error] truth=%s lio=%s", truth_topic_.c_str(), lio_topic_.c_str());
  }

private:
  void truthCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_truth_ = *msg;
    has_truth_ = true;
  }

  void lioCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!has_truth_) {
      return;
    }

    const auto & tp = latest_truth_.pose.pose.position;
    const auto & lp = msg->pose.pose.position;

    if (!has_reference_) {
      ref_truth_x_ = tp.x;
      ref_truth_y_ = tp.y;
      ref_truth_z_ = tp.z;
      ref_lio_x_ = lp.x;
      ref_lio_y_ = lp.y;
      ref_lio_z_ = lp.z;
      has_reference_ = true;
      RCLCPP_INFO(get_logger(),
        "[LIO Validation Error] reference set truth=(%.2f, %.2f, %.2f) lio=(%.2f, %.2f, %.2f)",
        ref_truth_x_, ref_truth_y_, ref_truth_z_, ref_lio_x_, ref_lio_y_, ref_lio_z_);
    }

    const double truth_x = compare_relative_motion_ ? tp.x - ref_truth_x_ : tp.x;
    const double truth_y = compare_relative_motion_ ? tp.y - ref_truth_y_ : tp.y;
    const double truth_z = compare_relative_motion_ ? tp.z - ref_truth_z_ : tp.z;
    const double lio_x = compare_relative_motion_ ? lp.x - ref_lio_x_ : lp.x;
    const double lio_y = compare_relative_motion_ ? lp.y - ref_lio_y_ : lp.y;
    const double lio_z = compare_relative_motion_ ? lp.z - ref_lio_z_ : lp.z;

    const double dx = lio_x - truth_x;
    const double dy = lio_y - truth_y;
    const double dz = lio_z - truth_z;
    const double err = std::sqrt(dx * dx + dy * dy + dz * dz);

    sum_sq_ += err * err;
    ++samples_;

    if (samples_ >= static_cast<uint64_t>(min_samples_)) {
      const double rmse = std::sqrt(sum_sq_ / static_cast<double>(samples_));
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), log_period_ms_,
        "[LIO Validation Error] samples=%lu err=%.3fm rmse=%.3fm truth=(%.2f, %.2f, %.2f) lio=(%.2f, %.2f, %.2f)",
        samples_, err, rmse, truth_x, truth_y, truth_z, lio_x, lio_y, lio_z);
    }
  }

  std::string truth_topic_;
  std::string lio_topic_;
  int log_period_ms_{1000};
  int min_samples_{5};
  bool compare_relative_motion_{true};
  bool has_truth_{false};
  bool has_reference_{false};
  uint64_t samples_{0};
  double sum_sq_{0.0};
  double ref_truth_x_{0.0};
  double ref_truth_y_{0.0};
  double ref_truth_z_{0.0};
  double ref_lio_x_{0.0};
  double ref_lio_y_{0.0};
  double ref_lio_z_{0.0};

  nav_msgs::msg::Odometry latest_truth_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LioValidationErrorNode>());
  rclcpp::shutdown();
  return 0;
}
