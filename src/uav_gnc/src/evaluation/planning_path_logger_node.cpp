#include <fstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

/*
 * planning_path_logger_node.cpp
 *
 * 이 파일은 UAV GNC 고도화 프로젝트 Week 8에서 생성된 D* Lite 경로를
 * CSV 파일로 저장하기 위한 ROS2 로깅 노드임.
 *
 * 기존 tracking_eval_node의 CSV에는 드론의 실제 위치(sim), 항법 추정 위치(nav),
 * 현재 추종 중인 목표 waypoint 정보는 저장되지만, path_planner_node가 실제로
 * 퍼블리시한 /planning/path 전체는 저장되지 않았음.
 *
 * 따라서 시각화 단계에서 CSV의 wp_x, wp_y, wp_z만 이용하면 실제 D* Lite가 만든
 * 우회 경로가 아니라 start-goal 기준의 단순 목표점처럼 보이는 문제가 있었음.
 * 이 파일은 그 문제를 해결하기 위해 /planning/path 토픽을 구독하고,
 * planner가 새 경로를 퍼블리시할 때마다 경로 안의 모든 pose를 CSV로 기록함.
 *
 * 기록되는 CSV는 각 경로 publish마다 path_id를 증가시키고,
 * 하나의 경로 안에 포함된 waypoint/keypoint들을 seq_idx 순서로 저장함.
 * 이를 통해 나중에 Python 시각화 코드에서 실제 D* Lite keypoints를 복원하여
 * XY trajectory와 3D trajectory 위에 함께 표시할 수 있음.
 *
 * 이 노드는 드론 제어, 항법, 경로계획 로직에는 직접 영향을 주지 않음.
 * 즉, /planning/path를 읽기만 하는 순수 logging/visualization 보조 노드임.
 *
 * 주요 입출력:
 *   Subscribe:
 *     - /planning/path       : path_planner_node가 퍼블리시하는 nav_msgs::msg::Path
 *
 *   Output:
 *     - planning_path_log.csv
 *       columns:
 *       path_id, msg_time_sec, recv_time_sec, seq_idx, x, y, z
 *
 * 사용 목적:
 *   - D* Lite가 실제로 생성한 경로/keypoints 기록
 *   - 장애물 회피 경로가 어떻게 만들어졌는지 시각화
 *   - README/포트폴리오용 XY/3D trajectory plot에서 실제 planner path 표시
 *   - “드론이 장애물을 피해 우회 경로를 생성했다”는 것을 정량/시각적으로 증명
 */

class PlanningPathLoggerNode : public rclcpp::Node
{
public:
  PlanningPathLoggerNode() : Node("planning_path_logger_node")
  {
    input_path_topic_ = this->declare_parameter<std::string>("input_path_topic", "/planning/path");
    csv_path_ = this->declare_parameter<std::string>("csv_path", "/home/lyj/uav_gnc_ws/planning_path_log.csv");
    append_ = this->declare_parameter<bool>("append", false);
    min_save_period_sec_ = this->declare_parameter<double>("min_save_period_sec", 0.0);

    std::ios_base::openmode mode = std::ios::out;
    if (append_) {
      mode |= std::ios::app;
    } else {
      mode |= std::ios::trunc;
    }

    csv_.open(csv_path_, mode);
    if (!csv_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "[Planning Path Logger] CSV open failed: %s", csv_path_.c_str());
      return;
    }

    if (!append_) {
      csv_ << "path_id,msg_time_sec,recv_time_sec,seq_idx,x,y,z\n";
      csv_.flush();
    }

    sub_ = this->create_subscription<nav_msgs::msg::Path>(
      input_path_topic_, 10,
      std::bind(&PlanningPathLoggerNode::pathCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "[Planning Path Logger] input=%s csv=%s append=%s min_period=%.2f",
      input_path_topic_.c_str(), csv_path_.c_str(), append_ ? "true" : "false", min_save_period_sec_);
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (!csv_.is_open() || msg->poses.empty()) {
      return;
    }

    const rclcpp::Time recv_time = this->now();
    const double recv_sec = recv_time.seconds();
    if (last_save_time_sec_ > 0.0 && min_save_period_sec_ > 0.0) {
      if ((recv_sec - last_save_time_sec_) < min_save_period_sec_) {
        return;
      }
    }
    last_save_time_sec_ = recv_sec;

    const double msg_sec = rclcpp::Time(msg->header.stamp).seconds();
    const int path_id = path_id_++;

    for (size_t i = 0; i < msg->poses.size(); ++i) {
      const auto & p = msg->poses[i].pose.position;
      csv_ << path_id << ","
           << msg_sec << ","
           << recv_sec << ","
           << i << ","
           << p.x << ","
           << p.y << ","
           << p.z << "\n";
    }
    csv_.flush();

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "[Planning Path Logger] saved path_id=%d, waypoints=%zu", path_id, msg->poses.size());
  }

  std::string input_path_topic_;
  std::string csv_path_;
  bool append_{false};
  double min_save_period_sec_{0.0};
  double last_save_time_sec_{-1.0};
  int path_id_{0};

  std::ofstream csv_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningPathLoggerNode>());
  rclcpp::shutdown();
  return 0;
}
