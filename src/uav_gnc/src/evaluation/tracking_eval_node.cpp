#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <iomanip>
#include <algorithm>

static double clamp01(double x) { return std::max(0.0, std::min(1.0, x)); }

class TrackingEvalNode : public rclcpp::Node
{
public:
  TrackingEvalNode() : Node("tracking_eval_node")
  {
    input_odom_topic_ = declare_parameter<std::string>("input_odom_topic", "/nav/odom");
    csv_path_         = declare_parameter<std::string>("csv_path", "tracking_eval.csv");
    rate_hz_          = declare_parameter<double>("rate_hz", 20.0);

    wp_x_ = declare_parameter<std::vector<double>>("waypoints_x", std::vector<double>{0.0});
    wp_y_ = declare_parameter<std::vector<double>>("waypoints_y", std::vector<double>{0.0});
    wp_z_ = declare_parameter<std::vector<double>>("waypoints_z", std::vector<double>{1.0});
    accept_radius_ = declare_parameter<double>("accept_radius", 0.5);

    auto_exit_on_complete_ = declare_parameter<bool>("auto_exit_on_complete", false);
    settle_time_sec_       = declare_parameter<double>("settle_time_sec", 0.0);

    if (wp_x_.size() != wp_y_.size() || wp_x_.size() != wp_z_.size() || wp_x_.size() < 2) {
      RCLCPP_FATAL(get_logger(), "Waypoints invalid: need same size (X, Y, Z) and at least 2 points.");
      throw std::runtime_error("Invalid waypoints");
    }

    csv_.open(csv_path_, std::ios::out);
    if (!csv_.is_open()) {
      RCLCPP_FATAL(get_logger(), "Failed to open csv_path: %s", csv_path_.c_str());
      throw std::runtime_error("CSV open failed");
    }
    
    // CSV 헤더에 cte_x, cte_y, cte_z 축별 에러 항목 추가함
    csv_ << "t_sec,x,y,z,seg_idx,wp_x,wp_y,wp_z,dist_to_wp,cross_track_err,cte_x,cte_y,cte_z,completed,time_to_complete_sec\n";
    csv_.flush();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_odom_topic_, 50,
      std::bind(&TrackingEvalNode::odomCallback, this, std::placeholders::_1));

    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TrackingEvalNode::onTimer, this));

    start_time_ = now();
  }

  ~TrackingEvalNode() override
  {
    finalizeAndPrint();
    if (csv_.is_open()) csv_.close();
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    z_ = msg->pose.pose.position.z; 
    have_odom_ = true;
  }

  // 인자에 축별 에러(cte_x, cte_y, cte_z)를 참조로 받아 반환하도록 수정함
  double crossTrackToSegment(size_t i, double px, double py, double pz, double &t_proj_out, double &cte_x, double &cte_y, double &cte_z)
  {
    const double x1 = wp_x_[i],     y1 = wp_y_[i],     z1 = wp_z_[i];
    const double x2 = wp_x_[i + 1], y2 = wp_y_[i + 1], z2 = wp_z_[i + 1];

    const double vx = x2 - x1, vy = y2 - y1, vz = z2 - z1;
    const double wx = px - x1, wy = py - y1, wz = pz - z1;

    const double vv = vx*vx + vy*vy + vz*vz;
    double t = 0.0;
    if (vv > 1e-12) t = (wx*vx + wy*vy + wz*vz) / vv;
    t = clamp01(t); 
    t_proj_out = t;

    const double projx = x1 + t * vx;
    const double projy = y1 + t * vy;
    const double projz = z1 + t * vz;

    // 투영점과 실제 기체 위치 사이의 축별 오차를 분리하여 저장함
    cte_x = px - projx;
    cte_y = py - projy;
    cte_z = pz - projz;

    return std::hypot(cte_x, cte_y, cte_z);
  }

  void advanceSegmentIfReached(double px, double py, double pz)
  {
    const size_t N = wp_x_.size();
    if (N == 0) return;
    
    double t_sec = (now() - start_time_).seconds();
    const double dist_to_last = std::hypot(wp_x_.back() - px, wp_y_.back() - py, wp_z_.back() - pz);

    if (seg_idx_ >= N - 2 && dist_to_last < accept_radius_ && !completed_) {
        reached_last_ = true;
        completed_ = true;
        complete_stamp_ = now();
        time_to_complete_sec_ = t_sec;
        return;
    }

    if (seg_idx_ + 1 >= N) return;
    const double wx = wp_x_[seg_idx_ + 1], wy = wp_y_[seg_idx_ + 1], wz = wp_z_[seg_idx_ + 1];
    
    const double d = std::hypot(wx - px, wy - py, wz - pz);
    if (d < accept_radius_) seg_idx_++;
  }

  void onTimer()
  {
    if (!have_odom_) return;

    if (!reached_last_) advanceSegmentIfReached(x_, y_, z_);

    const size_t N = wp_x_.size();
    const size_t seg_i = std::min(seg_idx_, N - 2);

    // 축별 CTE를 받을 변수 초기화 및 함수 호출함
    double tproj = 0.0, cte_x = 0.0, cte_y = 0.0, cte_z = 0.0;
    const double cte = crossTrackToSegment(seg_i, x_, y_, z_, tproj, cte_x, cte_y, cte_z);

    const double wx = wp_x_[seg_i + 1], wy = wp_y_[seg_i + 1], wz = wp_z_[seg_i + 1];
    const double dist_wp = std::hypot(wx - x_, wy - y_, wz - z_);
    const double t_sec = (now() - start_time_).seconds();

    count_++;
    sum_abs_ += std::abs(cte);
    sum_sq_  += cte * cte;
    max_abs_ = std::max(max_abs_, std::abs(cte));

    const int completed_int = completed_ ? 1 : 0;
    const double ttc = completed_ ? time_to_complete_sec_ : -1.0;

    // CSV 파일에 축별 오차(cte_x, cte_y, cte_z)를 추가로 기록함
    csv_ << std::fixed << std::setprecision(6)
        << t_sec << "," << x_ << "," << y_ << "," << z_ << ","
        << seg_i << "," << wx << "," << wy << "," << wz << ","
        << dist_wp << "," << cte << ","
        << cte_x << "," << cte_y << "," << cte_z << ","
        << completed_int << "," << ttc << "\n";

    if ((count_ % 50) == 0) csv_.flush();

    if (auto_exit_on_complete_ && completed_) {
      const double after_complete = (now() - complete_stamp_).seconds();
      if (after_complete >= settle_time_sec_) {
        csv_.flush();
        rclcpp::shutdown(); 
        return;
      }
    }
  }

  void finalizeAndPrint()
  {
    if (finalized_) return;
    finalized_ = true;
  }

private:
  std::string input_odom_topic_;
  std::string csv_path_;
  double rate_hz_{20.0};
  
  std::vector<double> wp_x_, wp_y_, wp_z_;
  double accept_radius_{0.5};

  bool have_odom_{false};
  double x_{0.0}, y_{0.0}, z_{0.0};
  rclcpp::Time start_time_;
  size_t seg_idx_{0};
  bool reached_last_{false};

  size_t count_{0};
  double sum_abs_{0.0}, sum_sq_{0.0}, max_abs_{0.0};
  bool finalized_{false};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::ofstream csv_;

  bool completed_{false};
  double time_to_complete_sec_{-1.0};
  bool auto_exit_on_complete_{false};
  double settle_time_sec_{0.0};   
  rclcpp::Time complete_stamp_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackingEvalNode>());
  rclcpp::shutdown();
  return 0;
}