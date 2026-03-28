#include <chrono>
#include <mutex>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <uav_gnc/sixdof.h>
#include <uav_gnc/controller.h>

using namespace std::chrono_literals;

// ======================================================================
// ControlNode
//
// use_mpc = false: Cascaded PID (기존 베이스라인)
// use_mpc = true:  Linear MPC + Reference Preview
//
// [Reference Preview 흐름]
//   guidance_node → /guidance/trajectory_preview (Float64MultiArray)
//   → MPCController::setTrajectoryPreview()
//   → update() 내부에서 preview Xref 사용
//   → guidance 속도에 맞춰 비행 (앞서가기/멈춤 현상 해소)
// ======================================================================
class ControlNode : public rclcpp::Node
{
public:
    ControlNode() : Node("control_node")
    {
        // ── 드론 물성치 ───────────────────────────────────────────────
        dt_ = this->declare_parameter<double>("dt", 0.01);

        params_.mass      = this->declare_parameter<double>("mass", 2.0);
        params_.inertia.x = this->declare_parameter<double>("Ix", 0.02);
        params_.inertia.y = this->declare_parameter<double>("Iy", 0.02);
        params_.inertia.z = this->declare_parameter<double>("Iz", 0.04);
        params_.g         = this->declare_parameter<double>("g", 9.80665);
        params_.use_drag  = this->declare_parameter<bool>("use_drag", true);
        params_.k1        = this->declare_parameter<double>("k1", 0.15);
        params_.k2        = this->declare_parameter<double>("k2", 0.02);

        // ── PID 게인 ─────────────────────────────────────────────────
        gains_.kp_pos_xy = this->declare_parameter<double>("kp_pos_xy", gains_.kp_pos_xy);
        gains_.kp_pos_z  = this->declare_parameter<double>("kp_pos_z",  gains_.kp_pos_z);
        gains_.kp_vel_xy = this->declare_parameter<double>("kp_vel_xy", gains_.kp_vel_xy);
        gains_.kp_vel_z  = this->declare_parameter<double>("kp_vel_z",  gains_.kp_vel_z);
        gains_.ki_vel_xy   = this->declare_parameter<double>("ki_vel_xy",   gains_.ki_vel_xy);
        gains_.ki_vel_z    = this->declare_parameter<double>("ki_vel_z",    gains_.ki_vel_z);
        gains_.max_int_vxy = this->declare_parameter<double>("max_int_vxy", gains_.max_int_vxy);
        gains_.max_int_vz  = this->declare_parameter<double>("max_int_vz",  gains_.max_int_vz);
        gains_.kp_att_rp  = this->declare_parameter<double>("kp_att_rp",  gains_.kp_att_rp);
        gains_.kd_att_rp  = this->declare_parameter<double>("kd_att_rp",  gains_.kd_att_rp);
        gains_.kp_att_yaw = this->declare_parameter<double>("kp_att_yaw", gains_.kp_att_yaw);
        gains_.kd_att_yaw = this->declare_parameter<double>("kd_att_yaw", gains_.kd_att_yaw);
        gains_.max_tilt_deg  = this->declare_parameter<double>("max_tilt_deg",  gains_.max_tilt_deg);
        gains_.max_vxy_cmd   = this->declare_parameter<double>("max_vxy_cmd",   gains_.max_vxy_cmd);
        gains_.max_axy_cmd   = this->declare_parameter<double>("max_axy_cmd",   gains_.max_axy_cmd);
        gains_.max_vz_cmd    = this->declare_parameter<double>("max_vz_cmd",    gains_.max_vz_cmd);
        gains_.max_az_cmd    = this->declare_parameter<double>("max_az_cmd",    gains_.max_az_cmd);
        gains_.thrust_min    = this->declare_parameter<double>("thrust_min",    gains_.thrust_min);
        gains_.thrust_max    = this->declare_parameter<double>("thrust_max",    gains_.thrust_max);
        gains_.moment_max_rp = this->declare_parameter<double>("moment_max_rp", gains_.moment_max_rp);
        gains_.moment_max_y  = this->declare_parameter<double>("moment_max_y",  gains_.moment_max_y);

        // ── MPC 초기화 ────────────────────────────────────────────────
        use_mpc_ = this->declare_parameter<bool>("use_mpc", false);

        if (use_mpc_) {
            mpc_params_.N        = this->declare_parameter<int>("mpc_N",        15);
            mpc_params_.q_pos_xy = this->declare_parameter<double>("mpc_q_pos_xy", 100.0);
            mpc_params_.q_pos_z  = this->declare_parameter<double>("mpc_q_pos_z",  200.0);
            mpc_params_.q_vel_xy = this->declare_parameter<double>("mpc_q_vel_xy", 10.0);
            mpc_params_.q_vel_z  = this->declare_parameter<double>("mpc_q_vel_z",  30.0);
            mpc_params_.r_acc_xy = this->declare_parameter<double>("mpc_r_acc_xy", 1.0);
            mpc_params_.r_acc_z  = this->declare_parameter<double>("mpc_r_acc_z",  0.5);

            mpc_controller_.init(mpc_params_, params_, gains_, dt_);

            // Preview 구독: guidance가 미래 N스텝 궤적을 퍼블리시하는 토픽
            // setTrajectoryPreview()로 MPC에 전달 → Xref 업데이트
            preview_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/guidance/trajectory_preview", 10,
                [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mtx_);
                    mpc_controller_.setTrajectoryPreview(msg->data);
                });

            RCLCPP_INFO(this->get_logger(),
                "MPC + Reference Preview mode (N=%d, q_pos_xy=%.1f, q_pos_z=%.1f)",
                mpc_params_.N, mpc_params_.q_pos_xy, mpc_params_.q_pos_z);
        } else {
            RCLCPP_INFO(this->get_logger(), "PID mode (Cascaded)");
        }

        // ── 퍼블리셔/구독자/타이머 ────────────────────────────────────
        wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
            "/control/wrench", 10);

        odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/nav/odom");
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

        sp_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/guidance/setpoint", 10,
            std::bind(&ControlNode::setpointCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_),
            std::bind(&ControlNode::onTimer, this));

        RCLCPP_INFO(this->get_logger(), "control_node started (dt=%.4f)", dt_);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        state_.p = { msg->pose.pose.position.x,
                     msg->pose.pose.position.y,
                     msg->pose.pose.position.z };
        state_.v = { msg->twist.twist.linear.x,
                     msg->twist.twist.linear.y,
                     msg->twist.twist.linear.z };
        state_.q = { msg->pose.pose.orientation.w,
                     msg->pose.pose.orientation.x,
                     msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z };
        state_.w = { msg->twist.twist.angular.x,
                     msg->twist.twist.angular.y,
                     msg->twist.twist.angular.z };
        has_state_ = true;
    }

    void setpointCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        ref_.p_ref = { msg->pose.pose.position.x,
                       msg->pose.pose.position.y,
                       msg->pose.pose.position.z };
        ref_.v_ref = { msg->twist.twist.linear.x,
                       msg->twist.twist.linear.y,
                       msg->twist.twist.linear.z };
        // guidance가 angular 공간에 숨겨서 보낸 가속도 피드포워드
        ref_.a_ref = { msg->twist.twist.angular.x,
                       msg->twist.twist.angular.y,
                       msg->twist.twist.angular.z };
        const auto& q = msg->pose.pose.orientation;
        ref_.yaw_ref = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        has_ref_ = true;
    }

    void onTimer()
    {
        State s;
        Ref   ref;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (!has_state_ || !has_ref_) return;
            if (std::isnan(state_.p.x) || std::isnan(state_.v.x) || std::isnan(state_.q.w)) return;
            s   = state_;
            ref = ref_;
        }

        Input u;
        if (use_mpc_) {
            u = mpc_controller_.update(s, ref);
        } else {
            u = controller_update(s, ref, params_, gains_, dt_, int_e_v_);
        }

        geometry_msgs::msg::WrenchStamped wrench;
        wrench.header.stamp    = this->now();
        wrench.header.frame_id = "base_link";
        wrench.wrench.force.x  = u.thrust_body.x;
        wrench.wrench.force.y  = u.thrust_body.y;
        wrench.wrench.force.z  = u.thrust_body.z;
        wrench.wrench.torque.x = u.moment_body.x;
        wrench.wrench.torque.y = u.moment_body.y;
        wrench.wrench.torque.z = u.moment_body.z;
        wrench_pub_->publish(wrench);
    }

    double          dt_{0.01};
    Params          params_;
    ControllerGains gains_;
    bool            use_mpc_{false};
    MPCParams       mpc_params_;
    MPCController   mpc_controller_;
    State           state_;
    Ref             ref_;
    bool            has_state_{false};
    bool            has_ref_{false};
    std::mutex      mtx_;
    std::string     odom_topic_;
    Vec3            int_e_v_{0.0, 0.0, 0.0};

    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        sp_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr preview_sub_;
    rclcpp::TimerBase::SharedPtr                                     timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}