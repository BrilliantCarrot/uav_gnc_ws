from __future__ import annotations

import argparse

import numpy as np

from uav_rl.envs import UavGoalEnv


def heuristic_action(obs: np.ndarray) -> np.ndarray:
    goal_error = obs[0:3] * 10.0
    v = obs[3:6] * 5.0
    horizontal_error = goal_error[:2]
    horizontal_dist = np.linalg.norm(horizontal_error)

    v_cmd = np.zeros(3, dtype=np.float32)
    v_cmd[2] = np.clip(0.8 * goal_error[2], -0.8, 0.8)
    if goal_error[2] < 0.7 and horizontal_dist > 1e-6:
        horizontal_dir = horizontal_error / horizontal_dist
        v_cmd[:2] = horizontal_dir * min(1.0, 0.35 * horizontal_dist)

    action = np.array([v_cmd[0] / 2.0, v_cmd[1] / 2.0, v_cmd[2] / 1.2, 0.0], dtype=np.float32)
    action[:3] -= np.array([v[0] / 10.0, v[1] / 10.0, v[2] / 7.0], dtype=np.float32)
    return np.clip(action, -1.0, 1.0)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a UAV RL environment smoke test.")
    parser.add_argument("--episodes", type=int, default=3)
    parser.add_argument("--steps", type=int, default=240)
    parser.add_argument("--random", action="store_true", help="Use random actions instead of a simple heuristic.")
    args = parser.parse_args()

    env = UavGoalEnv(seed=7)
    for ep in range(args.episodes):
        obs, info = env.reset()
        total_reward = 0.0
        terminated = False
        truncated = False
        last_info = info
        for _ in range(args.steps):
            if args.random:
                action = env.action_space.sample()
            elif env.cfg.use_guidance_prior:
                action = np.zeros(env.action_space.shape, dtype=np.float32)
            else:
                action = heuristic_action(obs)
            obs, reward, terminated, truncated, last_info = env.step(action)
            total_reward += reward
            if terminated or truncated:
                break

        p = last_info["position"]
        goal = last_info["goal"]
        print(
            f"episode={ep} reward={total_reward:.2f} "
            f"distance={last_info['distance']:.2f} "
            f"terminated={terminated} truncated={truncated} crashed={last_info['crashed']} "
            f"p=({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}) "
            f"goal=({goal[0]:.2f}, {goal[1]:.2f}, {goal[2]:.2f})"
        )


if __name__ == "__main__":
    main()
