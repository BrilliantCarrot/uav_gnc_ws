#include <algorithm>
#include <cmath>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class LioOdomTestSourceNode : public rclcpp::Node
{
public:
  LioOdomTestSourceNode() : Node("lio_odom_test_source_node")
  {
    truth_odom_topic_ = declare_parameter<std::string>("truth_odom_topic", "/sim/odom");
    output_odom_topic_ = declare_parameter<std::string>("output_odom_topic", "/lio/odom");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "world");
    output_child_frame_id_ = declare_parameter<std::string>("output_child_frame_id", "base_link");
    position_noise_std_ = declare_parameter<double>("position_noise_std", 0.03);
    velocity_noise_std_ = declare_parameter<double>("velocity_noise_std", 0.02);
    dropout_rate_ = std::clamp(declare_parameter<double>("dropout_rate", 0.0), 0.0, 1.0);
    random_seed_ = declare_parameter<int>("random_seed", 31);

    rng_.seed(static_cast<uint32_t>(random_seed_));
    pos_noise_ = std::normal_distribution<double>(0.0, std::max(0.0, position_noise_std_));
    vel_noise_ = std::normal_distribution<double>(0.0, std::max(0.0, velocity_noise_std_));
    dropout_dist_ = std::uniform_real_distribution<double>(0.0, 1.0);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      truth_odom_topic_, 10,
      std::bind(&LioOdomTestSourceNode::odomCallback, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 10);

    RCLCPP_WARN(get_logger(),
      "[LIO Odom Test Source] This is not LIO. It only validates the /lio/odom -> /nav/odom interface.");
  }

private:
  void fillCovariance(nav_msgs::msg::Odometry &msg) const
  {
    std::fill(msg.pose.covariance.begin(), msg.pose.covariance.end(), 0.0);
    const double p_var = position_noise_std_ * position_noise_std_;
    msg.pose.covariance[0] = p_var;
    msg.pose.covariance[7] = p_var;
    msg.pose.covariance[14] = p_var;
    msg.pose.covariance[21] = 0.01;
    msg.pose.covariance[28] = 0.01;
    msg.pose.covariance[35] = 0.03;

    std::fill(msg.twist.covariance.begin(), msg.twist.covariance.end(), 0.0);
    const double v_var = velocity_noise_std_ * velocity_noise_std_;
    msg.twist.covariance[0] = v_var;
    msg.twist.covariance[7] = v_var;
    msg.twist.covariance[14] = v_var;
    msg.twist.covariance[21] = 0.01;
    msg.twist.covariance[28] = 0.01;
    msg.twist.covariance[35] = 0.01;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (dropout_rate_ > 0.0 && dropout_dist_(rng_) < dropout_rate_) {
      return;
    }

    nav_msgs::msg::Odometry out = *msg;
    out.header.frame_id = output_frame_id_;
    out.child_frame_id = output_child_frame_id_;
    out.pose.pose.position.x += pos_noise_(rng_);
    out.pose.pose.position.y += pos_noise_(rng_);
    out.pose.pose.position.z += pos_noise_(rng_);
    out.twist.twist.linear.x += vel_noise_(rng_);
    out.twist.twist.linear.y += vel_noise_(rng_);
    out.twist.twist.linear.z += vel_noise_(rng_);
    fillCovariance(out);
    odom_pub_->publish(out);
  }

  std::string truth_odom_topic_;
  std::string output_odom_topic_;
  std::string output_frame_id_;
  std::string output_child_frame_id_;
  double position_noise_std_{0.03};
  double velocity_noise_std_{0.02};
  double dropout_rate_{0.0};
  int random_seed_{31};

  std::mt19937 rng_;
  std::normal_distribution<double> pos_noise_;
  std::normal_distribution<double> vel_noise_;
  std::uniform_real_distribution<double> dropout_dist_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LioOdomTestSourceNode>());
  rclcpp::shutdown();
  return 0;
}
