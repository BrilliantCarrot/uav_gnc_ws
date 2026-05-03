#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "uav_gnc/dstar_lite.h"

class PathPlannerNode : public rclcpp::Node
{
public:
  PathPlannerNode() : Node("path_planner_node")
  {
    const double width_m = this->declare_parameter<double>("grid_width_m", 30.0);
    const double height_m = this->declare_parameter<double>("grid_height_m", 30.0);
    const double res = this->declare_parameter<double>("grid_resolution", 0.5);
    const double origin_x = this->declare_parameter<double>("grid_origin_x", -10.0);
    const double origin_y = this->declare_parameter<double>("grid_origin_y", -10.0);

    fly_alt_ = this->declare_parameter<double>("fly_altitude", 2.0);
    goal_x_ = this->declare_parameter<double>("goal_x", 6.0);
    goal_y_ = this->declare_parameter<double>("goal_y", 8.0);
    replan_rate_hz_ = this->declare_parameter<double>("replan_rate_hz", 2.0);
    use_dynamic_occupancy_ = this->declare_parameter<bool>("use_dynamic_occupancy", true);
    replan_on_map_change_ = this->declare_parameter<bool>("replan_on_map_change", true);
    replan_on_start_move_m_ = this->declare_parameter<double>("replan_on_start_move_m", 0.5);
    occupancy_threshold_ = this->declare_parameter<int>("occupancy_threshold", 50);

    const auto obs_x = this->declare_parameter<std::vector<double>>(
    "obstacle_x", std::vector<double>{});

    const auto obs_y = this->declare_parameter<std::vector<double>>(
    "obstacle_y", std::vector<double>{});

    const auto obs_r = this->declare_parameter<std::vector<double>>(
    "obstacle_r", std::vector<double>{});

    grid_w_ = static_cast<int>(std::round(width_m / res));
    grid_h_ = static_cast<int>(std::round(height_m / res));

    dstar_.init(grid_w_, grid_h_, res, origin_x, origin_y);

    static_occ_.assign(static_cast<size_t>(grid_w_ * grid_h_), false);
    dynamic_occ_.assign(static_cast<size_t>(grid_w_ * grid_h_), false);
    applied_occ_.assign(static_cast<size_t>(grid_w_ * grid_h_), false);

    const size_t n_obs = std::min({obs_x.size(), obs_y.size(), obs_r.size()});
    for (size_t i = 0; i < n_obs; ++i) {
      markStaticCircleObstacle(obs_x[i], obs_y[i], obs_r[i]);
    }

    dstar_.setStart(0.0, 0.0);
    dstar_.setGoal(goal_x_, goal_y_);

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/path", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/planning/grid_markers", 1);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/nav/odom", 10,
      std::bind(&PathPlannerNode::odomCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/planning/goal", 10,
      std::bind(&PathPlannerNode::goalCallback, this, std::placeholders::_1));

    if (use_dynamic_occupancy_) {
      occupancy_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/planning/occupancy_updates", 10,
        std::bind(&PathPlannerNode::occupancyCallback, this, std::placeholders::_1));
    }

    const int period_ms = static_cast<int>(1000.0 / replan_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&PathPlannerNode::planningLoop, this));

    RCLCPP_INFO(this->get_logger(),
      "[Planner] Grid=%dx%d res=%.2f goal=(%.2f, %.2f) static_obs=%zu dynamic_occ=%s",
      grid_w_, grid_h_, res, goal_x_, goal_y_, n_obs,
      use_dynamic_occupancy_ ? "on" : "off");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    cur_x_ = msg->pose.pose.position.x;
    cur_y_ = msg->pose.pose.position.y;
    odom_received_ = true;
  }

  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    goal_x_ = msg->point.x;
    goal_y_ = msg->point.y;
    dstar_.setGoal(goal_x_, goal_y_);
    need_replan_ = true;

    RCLCPP_INFO(this->get_logger(),
      "[Planner] 새 목표점 받음: (%.2f, %.2f)", goal_x_, goal_y_);
  }

  void occupancyCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (static_cast<int>(msg->info.width) != grid_w_ || static_cast<int>(msg->info.height) != grid_h_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[Planner] occupancy grid 크기 불일치. planner=%dx%d, msg=%ux%u",
        grid_w_, grid_h_, msg->info.width, msg->info.height);
      return;
    }

    if (std::fabs(msg->info.resolution - dstar_.getResolution()) > 1e-6) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[Planner] occupancy grid resolution 불일치. planner=%.3f, msg=%.3f",
        dstar_.getResolution(), msg->info.resolution);
      return;
    }

    size_t changed_cells = 0;

    for (int yi = 0; yi < grid_h_; ++yi) {
      for (int xi = 0; xi < grid_w_; ++xi) {
        const size_t idx = toIndex(xi, yi);
        const bool dyn_occ = msg->data[idx] >= occupancy_threshold_;
        dynamic_occ_[idx] = dyn_occ;

        const bool combined_occ = static_occ_[idx] || dynamic_occ_[idx];
        if (combined_occ == applied_occ_[idx]) {
          continue;
        }

        const bool changed = dstar_.updateCell(xi, yi, combined_occ);
        applied_occ_[idx] = combined_occ;
        if (changed) {
          changed_cells++;
        }
      }
    }

    if (changed_cells > 0 && replan_on_map_change_) {
      need_replan_ = true;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[Planner] occupancy 변화 감지: changed_cells=%zu", changed_cells);
    }
  }

  void planningLoop()
  {
    if (!odom_received_) {
      return;
    }

    dstar_.setStart(cur_x_, cur_y_);

    const double moved_dist = std::hypot(cur_x_ - last_plan_x_, cur_y_ - last_plan_y_);
    const bool start_moved = !has_last_plan_start_ || moved_dist >= replan_on_start_move_m_;

    if (!need_replan_ && !start_moved) {
      return;
    }

    const bool success = dstar_.plan();
    if (!success) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[Planner] 경로 없음: start=(%.2f, %.2f) goal=(%.2f, %.2f)",
        cur_x_, cur_y_, goal_x_, goal_y_);
      publishObstacleMarkers();
      return;
    }

    const auto raw_path = dstar_.getPath();
    const auto keypoints = extractKeypoints(raw_path, 1.5);
    publishPath(keypoints);
    publishObstacleMarkers();

    last_plan_x_ = cur_x_;
    last_plan_y_ = cur_y_;
    has_last_plan_start_ = true;
    need_replan_ = false;
  }

  std::vector<std::array<double, 2>> extractKeypoints(
    const std::vector<std::array<double, 2>> & raw,
    double min_dist) const
  {
    if (raw.size() <= 2) {
      return raw;
    }

    std::vector<std::array<double, 2>> out;
    out.push_back(raw.front());

    for (size_t i = 1; i + 1 < raw.size(); ++i) {
      const double dx1 = raw[i][0] - raw[i - 1][0];
      const double dy1 = raw[i][1] - raw[i - 1][1];
      const double dx2 = raw[i + 1][0] - raw[i][0];
      const double dy2 = raw[i + 1][1] - raw[i][1];

      const double len1 = std::hypot(dx1, dy1);
      const double len2 = std::hypot(dx2, dy2);
      if (len1 < 1e-6 || len2 < 1e-6) {
        continue;
      }

      double cos_angle = (dx1 * dx2 + dy1 * dy2) / (len1 * len2);
      cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
      const double angle_deg = std::acos(cos_angle) * 180.0 / M_PI;

      const auto & last = out.back();
      const double dist = std::hypot(raw[i][0] - last[0], raw[i][1] - last[1]);

      if (angle_deg > 20.0 || dist >= min_dist) {
        out.push_back(raw[i]);
      }
    }

    out.push_back(raw.back());
    return out;
  }

  void publishPath(const std::vector<std::array<double, 2>> & wp_list)
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = "world";

    for (const auto & wp : wp_list) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path_msg.header;
      ps.pose.position.x = wp[0];
      ps.pose.position.y = wp[1];
      ps.pose.position.z = fly_alt_;
      ps.pose.orientation.w = 1.0;
      path_msg.poses.push_back(ps);
    }

    path_pub_->publish(path_msg);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "[Planner] 경로 퍼블리시: %zu waypoints", wp_list.size());
  }

  void publishObstacleMarkers()
  {
    visualization_msgs::msg::MarkerArray ma;

    visualization_msgs::msg::Marker del;
    del.header.frame_id = "world";
    del.header.stamp = this->get_clock()->now();
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);

    visualization_msgs::msg::Marker cubes;
    cubes.header.frame_id = "world";
    cubes.header.stamp = this->get_clock()->now();
    cubes.ns = "planner_obstacles";
    cubes.id = 0;
    cubes.type = visualization_msgs::msg::Marker::CUBE_LIST;
    cubes.action = visualization_msgs::msg::Marker::ADD;
    cubes.pose.orientation.w = 1.0;
    cubes.scale.x = dstar_.getResolution();
    cubes.scale.y = dstar_.getResolution();
    cubes.scale.z = fly_alt_;
    cubes.color.r = 1.0f;
    cubes.color.g = 0.2f;
    cubes.color.b = 0.2f;
    cubes.color.a = 0.6f;

    for (int yi = 0; yi < grid_h_; ++yi) {
      for (int xi = 0; xi < grid_w_; ++xi) {
        if (!dstar_.isOccupied(xi, yi)) {
          continue;
        }

        const auto wp = dstar_.cellToWorld({xi, yi});
        geometry_msgs::msg::Point p;
        p.x = wp[0];
        p.y = wp[1];
        p.z = fly_alt_ * 0.5;
        cubes.points.push_back(p);
      }
    }

    ma.markers.push_back(cubes);
    marker_pub_->publish(ma);
  }

  void markStaticCircleObstacle(double cx, double cy, double radius)
  {
    dstar_.markCircleObstacle(cx, cy, radius);

    const Cell center = dstar_.worldToCell(cx, cy);
    const int r_cells = static_cast<int>(std::ceil(radius / dstar_.getResolution())) + 1;

    for (int dy = -r_cells; dy <= r_cells; ++dy) {
      for (int dx = -r_cells; dx <= r_cells; ++dx) {
        const int xi = center.x + dx;
        const int yi = center.y + dy;
        if (!dstar_.inBounds(xi, yi)) {
          continue;
        }

        const auto wp = dstar_.cellToWorld({xi, yi});
        if (std::hypot(wp[0] - cx, wp[1] - cy) <= radius) {
          const size_t idx = toIndex(xi, yi);
          static_occ_[idx] = true;
          applied_occ_[idx] = true;
        }
      }
    }
  }

  size_t toIndex(int xi, int yi) const
  {
    return static_cast<size_t>(yi * grid_w_ + xi);
  }

  DStarLite dstar_;

  int grid_w_{0};
  int grid_h_{0};

  double cur_x_{0.0};
  double cur_y_{0.0};
  bool odom_received_{false};

  double goal_x_{6.0};
  double goal_y_{8.0};
  double fly_alt_{2.0};
  double replan_rate_hz_{2.0};
  bool use_dynamic_occupancy_{true};
  bool replan_on_map_change_{true};
  double replan_on_start_move_m_{0.5};
  int occupancy_threshold_{50};

  bool need_replan_{true};
  bool has_last_plan_start_{false};
  double last_plan_x_{0.0};
  double last_plan_y_{0.0};

  std::vector<bool> static_occ_;
  std::vector<bool> dynamic_occ_;
  std::vector<bool> applied_occ_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}