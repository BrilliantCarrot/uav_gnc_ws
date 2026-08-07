from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def _yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _is_finite_vec(values) -> bool:
    return all(math.isfinite(float(v)) for v in values)


class RlResidualGuidanceNode(Node):
    """Deploy a trained PPO residual policy inside the ROS2 GNC pipeline.

    This node is for integration validation, not training. If no model is
    configured or loading fails, it publishes the original guidance reference
    unchanged so the launch can be tested in passthrough mode first.
    """

    def __init__(self) -> None:
        super().__init__("rl_residual_guidance_node")

        self.nav_odom_topic = self.declare_parameter("nav_odom_topic", "/nav/odom").value
        self.input_setpoint_topic = self.declare_parameter(
            "input_setpoint_topic", "/guidance/setpoint"
        ).value
        self.output_setpoint_topic = self.declare_parameter(
            "output_setpoint_topic", "/guidance/setpoint_rl"
        ).value
        self.input_preview_topic = self.declare_parameter(
            "input_preview_topic", "/guidance/trajectory_preview"
        ).value
        self.output_preview_topic = self.declare_parameter(
            "output_preview_topic", "/guidance/trajectory_preview_rl"
        ).value
        self.model_path = self.declare_parameter("model_path", "").value
        self.enable_policy = bool(self.declare_parameter("enable_policy", False).value)
        self.max_vel_residual_xy = float(
            self.declare_parameter("max_vel_residual_xy", 0.30).value
        )
        self.max_vel_residual_z = float(
            self.declare_parameter("max_vel_residual_z", 0.20).value
        )
        self.max_yaw_rate_residual = float(
            self.declare_parameter("max_yaw_rate_residual", 0.20).value
        )
        self.max_setpoint_age_sec = float(
            self.declare_parameter("max_setpoint_age_sec", 0.50).value
        )
        self.log_period_ms = int(self.declare_parameter("log_period_ms", 1000).value)

        self.model = self._load_model()
        self.last_nav: Odometry | None = None
        self.last_policy_action = np.zeros(4, dtype=np.float64)
        self.last_residual = np.zeros(4, dtype=np.float64)

        self.setpoint_pub = self.create_publisher(Odometry, self.output_setpoint_topic, 10)
        self.preview_pub = self.create_publisher(Float64MultiArray, self.output_preview_topic, 10)
        self.action_pub = self.create_publisher(Float64MultiArray, "/rl/action", 10)

        self.create_subscription(Odometry, self.nav_odom_topic, self._nav_cb, 10)
        self.create_subscription(Odometry, self.input_setpoint_topic, self._setpoint_cb, 10)
        self.create_subscription(Float64MultiArray, self.input_preview_topic, self._preview_cb, 10)

        mode = "policy" if self.model is not None else "passthrough"
        self.get_logger().info(
            f"rl_residual_guidance_node started mode={mode} "
            f"input={self.input_setpoint_topic} output={self.output_setpoint_topic}"
        )

    def _load_model(self):
        if not self.enable_policy:
            self.get_logger().info("RL policy disabled; using passthrough mode.")
            return None
        if not self.model_path:
            self.get_logger().warn("enable_policy=true but model_path is empty; using passthrough.")
            return None
        path = Path(str(self.model_path)).expanduser()
        if not path.exists():
            self.get_logger().warn(f"RL model not found: {path}; using passthrough.")
            return None
        try:
            from stable_baselines3 import PPO
        except ImportError:
            self.get_logger().warn("stable_baselines3 is not installed; using passthrough.")
            return None
        try:
            return PPO.load(str(path), device="cpu")
        except Exception as exc:  # noqa: BLE001 - keep flight stack alive on load failure.
            self.get_logger().warn(f"Failed to load RL model '{path}': {exc}; using passthrough.")
            return None

    def _nav_cb(self, msg: Odometry) -> None:
        self.last_nav = msg

    def _preview_cb(self, msg: Float64MultiArray) -> None:
        # First integration step keeps MPC preview consistent with the selected
        # control topic. Future work can apply the same residual over preview.
        self.preview_pub.publish(msg)

    def _setpoint_cb(self, msg: Odometry) -> None:
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist

        action = np.zeros(4, dtype=np.float64)
        if self.model is not None and self.last_nav is not None and self._fresh(msg):
            obs = self._make_observation(self.last_nav, msg)
            if obs is not None:
                try:
                    raw_action, _ = self.model.predict(obs, deterministic=True)
                    action = np.asarray(raw_action, dtype=np.float64).reshape(-1)[:4]
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f"RL inference failed; passthrough this setpoint: {exc}")
                    action = np.zeros(4, dtype=np.float64)

        if not _is_finite_vec(action) or action.shape[0] != 4:
            action = np.zeros(4, dtype=np.float64)
        action = np.clip(action, -1.0, 1.0)

        out.twist.twist.linear.x += float(action[0] * self.max_vel_residual_xy)
        out.twist.twist.linear.y += float(action[1] * self.max_vel_residual_xy)
        out.twist.twist.linear.z += float(action[2] * self.max_vel_residual_z)
        # The existing control interface stores acceleration feed-forward in
        # twist.angular. Keep it unchanged and expose yaw-rate residual only in
        # /rl/action until a dedicated yaw-rate interface is added.
        yaw_rate_residual = float(action[3] * self.max_yaw_rate_residual)
        self.last_policy_action = action.copy()
        self.last_residual = np.array(
            [
                action[0] * self.max_vel_residual_xy,
                action[1] * self.max_vel_residual_xy,
                action[2] * self.max_vel_residual_z,
                yaw_rate_residual,
            ],
            dtype=np.float64,
        )

        self.setpoint_pub.publish(out)
        action_msg = Float64MultiArray()
        action_msg.data = self.last_residual.tolist()
        self.action_pub.publish(action_msg)

        e = self._goal_error(self.last_nav, msg) if self.last_nav is not None else np.zeros(3)
        mode = "policy" if self.model is not None else "passthrough"
        self.get_logger().info(
            f"[RL Guidance] mode={mode} "
            f"e=({e[0]:.2f}, {e[1]:.2f}, {e[2]:.2f}) "
            f"vel_res=({self.last_residual[0]:.2f}, {self.last_residual[1]:.2f}, {self.last_residual[2]:.2f}) "
            f"yaw_rate_res={self.last_residual[3]:.2f}",
            throttle_duration_sec=self.log_period_ms / 1000.0,
        )

    def _fresh(self, msg: Odometry) -> bool:
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return True
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        return (now - stamp_sec) <= self.max_setpoint_age_sec

    def _make_observation(self, nav: Odometry, sp: Odometry) -> np.ndarray | None:
        e = self._goal_error(nav, sp)
        v = np.array(
            [
                nav.twist.twist.linear.x,
                nav.twist.twist.linear.y,
                nav.twist.twist.linear.z,
            ],
            dtype=np.float64,
        )
        q = np.array(
            [
                nav.pose.pose.orientation.w,
                nav.pose.pose.orientation.x,
                nav.pose.pose.orientation.y,
                nav.pose.pose.orientation.z,
            ],
            dtype=np.float64,
        )
        w = np.array(
            [
                nav.twist.twist.angular.x,
                nav.twist.twist.angular.y,
                nav.twist.twist.angular.z,
            ],
            dtype=np.float64,
        )
        dist = np.linalg.norm(e)
        obs = np.concatenate(
            [
                e / 10.0,
                v / 5.0,
                q,
                w / 5.0,
                np.array([dist / 10.0], dtype=np.float64),
                np.clip(self.last_policy_action, -1.0, 1.0),
            ]
        )
        if not _is_finite_vec(obs) or obs.shape[0] != 18:
            return None
        return obs.astype(np.float32)

    @staticmethod
    def _goal_error(nav: Odometry, sp: Odometry) -> np.ndarray:
        return np.array(
            [
                sp.pose.pose.position.x - nav.pose.pose.position.x,
                sp.pose.pose.position.y - nav.pose.pose.position.y,
                sp.pose.pose.position.z - nav.pose.pose.position.z,
            ],
            dtype=np.float64,
        )


def main() -> None:
    rclpy.init()
    node = RlResidualGuidanceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
