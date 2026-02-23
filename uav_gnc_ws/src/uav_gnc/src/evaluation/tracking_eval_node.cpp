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
    accept_radius_ = declare_parameter<double>("accept_radius", 0.5);

    auto_exit_on_complete_ = declare_parameter<bool>("auto_exit_on_complete", false);
    settle_time_sec_       = declare_parameter<double>("settle_time_sec", 0.0);

    if (wp_x_.size() != wp_y_.size() || wp_x_.size() < 2) {
      RCLCPP_FATAL(get_logger(), "Waypoints invalid: need same size and at least 2 points.");
      throw std::runtime_error("Invalid waypoints");
    }

    // CSV open
    csv_.open(csv_path_, std::ios::out);
    if (!csv_.is_open()) {
      RCLCPP_FATAL(get_logger(), "Failed to open csv_path: %s", csv_path_.c_str());
      throw std::runtime_error("CSV open failed");
    }
    csv_ << "t_sec,x,y,seg_idx,wp_x,wp_y,dist_to_wp,cross_track_err,completed,time_to_complete_sec\n";

    csv_.flush();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_odom_topic_, 50,
      std::bind(&TrackingEvalNode::odomCallback, this, std::placeholders::_1));

    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TrackingEvalNode::onTimer, this));

    start_time_ = now();

    RCLCPP_INFO(get_logger(),
      "TrackingEvalNode started. in=%s, csv=%s, Nwp=%zu",
      input_odom_topic_.c_str(), csv_path_.c_str(), wp_x_.size());
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
    have_odom_ = true;
  }

  // Cross-track error to current segment (wp[i] -> wp[i+1])
  double crossTrackToSegment(size_t i, double px, double py, double &t_proj_out)
  {
    const double x1 = wp_x_[i],     y1 = wp_y_[i];
    const double x2 = wp_x_[i + 1], y2 = wp_y_[i + 1];

    const double vx = x2 - x1;
    const double vy = y2 - y1;
    const double wx = px - x1;
    const double wy = py - y1;

    const double vv = vx*vx + vy*vy;
    double t = 0.0;
    if (vv > 1e-12) t = (wx*vx + wy*vy) / vv;
    t = clamp01(t);
    t_proj_out = t;

    const double projx = x1 + t * vx;
    const double projy = y1 + t * vy;

    return std::hypot(px - projx, py - projy);
  }

void advanceSegmentIfReached(double px, double py)
  {
    const size_t N = wp_x_.size();
    if (N == 0) return;
    // 현재 비행 시간 계산
    double t_sec = (now() - start_time_).seconds();
    // 출발점과 도착점이 같을 때 바로 종료됨을 막기 위해, 비행 시작 후 최소 5초가 지난 시점부터 최종 목적지 도달 검사
    const double dist_to_last = std::hypot(wp_x_.back() - px, wp_y_.back() - py);

    // 단순 시간(5초)이 아니라, 현재 진행 중인 구간(seg_idx_)이 
    // 전체 웨이포인트의 마지막 구간(N - 2)일 때만 최종 목적지 도달을 검사함
    if (seg_idx_ >= N - 2 && dist_to_last < accept_radius_ && !completed_) {
        reached_last_ = true;
        completed_ = true;
        complete_stamp_ = now();
        time_to_complete_sec_ = t_sec;
        RCLCPP_INFO(get_logger(),
          "MISSION COMPLETE! Reached final destination. time=%.3f",
          time_to_complete_sec_);
        return;
    }

    // 중간 경유지 체크 로직 (코너를 크게 돌아서 못 찍더라도 에러 안 나게 패스)
    if (seg_idx_ + 1 >= N) return;
    const double wx = wp_x_[seg_idx_ + 1];
    const double wy = wp_y_[seg_idx_ + 1];
    const double d = std::hypot(wx - px, wy - py);
    if (d < accept_radius_) {
        seg_idx_++;
    }
  }

  void onTimer()
  {
    if (!have_odom_) return;

    // segment update
    if (!reached_last_) advanceSegmentIfReached(x_, y_);

    const size_t N = wp_x_.size();
    const size_t seg_i = std::min(seg_idx_, N - 2);

    double tproj = 0.0;
    const double cte = crossTrackToSegment(seg_i, x_, y_, tproj);

    // distance to current target waypoint (seg_i+1)
    const double wx = wp_x_[seg_i + 1];
    const double wy = wp_y_[seg_i + 1];
    const double dist_wp = std::hypot(wx - x_, wy - y_);

    // time
    const double t_sec = (now() - start_time_).seconds();

    // accumulate stats
    count_++;
    sum_abs_ += std::abs(cte);
    sum_sq_  += cte * cte;
    max_abs_ = std::max(max_abs_, std::abs(cte));

    // csv
    const int completed_int = completed_ ? 1 : 0;
    const double ttc = completed_ ? time_to_complete_sec_ : -1.0;

    csv_ << std::fixed << std::setprecision(6)
        << t_sec << "," << x_ << "," << y_ << ","
        << seg_i << "," << wx << "," << wy << ","
        << dist_wp << "," << cte << ","
        << completed_int << "," << ttc << "\n";

    // flush occasionally (not every tick)
    if ((count_ % 50) == 0) csv_.flush();

    // print occasionally
    if ((count_ % 200) == 0) {
      const double mean_abs = (count_ > 0) ? (sum_abs_ / count_) : 0.0;
      const double rmse = (count_ > 0) ? std::sqrt(sum_sq_ / count_) : 0.0;
      RCLCPP_INFO(get_logger(),
        "cte stats: mean_abs=%.3f, rmse=%.3f, max_abs=%.3f (N=%zu)%s",
        mean_abs, rmse, max_abs_, count_, reached_last_ ? " [reached_last]" : "");
    }

    if (auto_exit_on_complete_ && completed_) {
      // 완주 후 settle_time_sec 동안은 계속 기록하고 그 다음 종료
      const double after_complete = (now() - complete_stamp_).seconds();
      if (after_complete >= settle_time_sec_) {
        csv_.flush();
        RCLCPP_INFO(get_logger(), "Auto-exit eval node after completion (settle=%.2f sec).", settle_time_sec_);
        rclcpp::shutdown(); // 이 프로세스(평가 노드)만 종료
        return;
      }
    }
  }

  void finalizeAndPrint()
  {
    if (finalized_) return;
    finalized_ = true;

    if (count_ == 0) {
      RCLCPP_WARN(get_logger(), "No samples collected.");
      return;
    }
    const double mean_abs = sum_abs_ / count_;
    const double rmse = std::sqrt(sum_sq_ / count_);

    RCLCPP_INFO(get_logger(),
      "FINAL cte stats: mean_abs=%.4f, rmse=%.4f, max_abs=%.4f (N=%zu). CSV saved: %s",
      mean_abs, rmse, max_abs_, count_, csv_path_.c_str());
  }

private:
  // params
  std::string input_odom_topic_;
  std::string csv_path_;
  double rate_hz_{20.0};
  std::vector<double> wp_x_, wp_y_;
  double accept_radius_{0.5};

  // state
  bool have_odom_{false};
  double x_{0.0}, y_{0.0};
  rclcpp::Time start_time_;
  size_t seg_idx_{0};
  bool reached_last_{false};

  // stats
  size_t count_{0};
  double sum_abs_{0.0};
  double sum_sq_{0.0};
  double max_abs_{0.0};
  bool finalized_{false};

  // ros
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // csv
  std::ofstream csv_;

  bool completed_{false};
  double time_to_complete_sec_{-1.0};

  bool auto_exit_on_complete_{false};
  double settle_time_sec_{0.0};   // 완주 후 몇 초 더 기록하고 종료할지
  rclcpp::Time complete_stamp_;

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackingEvalNode>());
  rclcpp::shutdown();
  return 0;
}
