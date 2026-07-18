#include <algorithm>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuLioAdapterNode : public rclcpp::Node
{
public:
  ImuLioAdapterNode() : Node("imu_lio_adapter_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/sim/imu");
    output_topic_ = declare_parameter<std::string>("output_topic", "/lio/imu");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "imu");

    accel_noise_std_ = declare_parameter<double>("accel_noise_std", 0.05);
    gyro_noise_std_ = declare_parameter<double>("gyro_noise_std", 0.005);
    orientation_covariance_ = declare_parameter<double>("orientation_covariance", 0.01);
    preserve_input_frame_ = declare_parameter<bool>("preserve_input_frame", false);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ImuLioAdapterNode::imuCallback, this, std::placeholders::_1));
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, 50);

    RCLCPP_INFO(get_logger(),
      "[IMU LIO Adapter] input=%s output=%s frame=%s",
      input_topic_.c_str(), output_topic_.c_str(), output_frame_id_.c_str());
  }

private:
  void fillDiagonal(double *cov, double value)
  {
    std::fill(cov, cov + 9, 0.0);
    cov[0] = value;
    cov[4] = value;
    cov[8] = value;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    sensor_msgs::msg::Imu out = *msg;
    if (!preserve_input_frame_) {
      out.header.frame_id = output_frame_id_;
    }

    fillDiagonal(out.orientation_covariance.data(), orientation_covariance_);
    fillDiagonal(out.angular_velocity_covariance.data(), gyro_noise_std_ * gyro_noise_std_);
    fillDiagonal(out.linear_acceleration_covariance.data(), accel_noise_std_ * accel_noise_std_);
    imu_pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  double accel_noise_std_{0.05};
  double gyro_noise_std_{0.005};
  double orientation_covariance_{0.01};
  bool preserve_input_frame_{false};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuLioAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
