from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

from uav_rl.envs import UavGoalEnv, UavGoalEnvConfig
from uav_rl.scripts.check_env import heuristic_action


TASK_GOALS = {
    "easy": ((-1.5, 1.5, 1.6), (1.5, 3.0, 2.1)),
    "medium": ((-3.0, 2.0, 1.5), (3.0, 5.0, 2.3)),
    "full": ((-5.0, 3.0, 1.5), (5.0, 8.0, 2.5)),
}


def main() -> None:
    parser = argparse.ArgumentParser(description="Roll out a trained UAV PPO policy or heuristic baseline.")
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--episodes", type=int, default=5)
    parser.add_argument("--max-steps", type=int, default=260)
    parser.add_argument("--task", choices=TASK_GOALS.keys(), default="medium")
    parser.add_argument("--save-dir", type=Path, default=None)
    parser.add_argument("--max-saved-episodes", type=int, default=5)
    parser.add_argument("--save-failures", action="store_true")
    args = parser.parse_args()

    model = None
    if args.model is not None:
        try:
            from stable_baselines3 import PPO
        except ImportError as exc:
            raise SystemExit("stable-baselines3 is required to load a trained model.") from exc
        model = PPO.load(str(args.model), device="cpu")

    goal_low, goal_high = TASK_GOALS[args.task]
    env = UavGoalEnv(config=UavGoalEnvConfig(goal_low=goal_low, goal_high=goal_high), seed=123)
    if args.save_dir is not None:
        args.save_dir.mkdir(parents=True, exist_ok=True)

    distances = []
    successes = 0
    crashes = 0
    rewards = []
    saved = 0
    for ep in range(args.episodes):
        obs, info = env.reset()
        total_reward = 0.0
        last_info = info
        terminated = False
        truncated = False
        rows = [_make_row(ep, -1, 0.0, info, np.zeros(4), np.zeros(4), 0.0, 0.0, False, False, False)]
        for step in range(args.max_steps):
            if model is None:
                raw_action = np.zeros(env.action_space.shape, dtype=np.float32) if env.cfg.use_guidance_prior else heuristic_action(obs)
            else:
                raw_action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, last_info = env.step(raw_action)
            total_reward += reward
            rows.append(
                _make_row(
                    ep,
                    step,
                    float(last_info["t"]),
                    last_info,
                    np.asarray(raw_action, dtype=np.float64),
                    np.asarray(last_info["applied_action"], dtype=np.float64),
                    float(reward),
                    float(total_reward),
                    bool(terminated and not last_info["crashed"]),
                    bool(last_info["crashed"]),
                    bool(truncated),
                )
            )
            if terminated or truncated:
                break

        success = terminated and not last_info["crashed"]
        distances.append(last_info["distance"])
        successes += int(success)
        crashes += int(last_info["crashed"])
        rewards.append(total_reward)
        if args.save_dir is not None and saved < args.max_saved_episodes and (success or args.save_failures):
            label = "success" if success else "failure"
            csv_path = args.save_dir / f"{label}_episode_{ep:03d}.csv"
            _write_csv(csv_path, rows)
            saved += 1
            print(f"saved_rollout={csv_path}")
        print(
            f"episode={ep} reward={total_reward:.2f} distance={last_info['distance']:.2f} "
            f"success={success} crashed={last_info['crashed']}"
        )

    print(
        f"summary success_rate={successes / max(args.episodes, 1):.2f} "
        f"crash_rate={crashes / max(args.episodes, 1):.2f} "
        f"mean_final_distance={float(np.mean(distances)):.2f}m "
        f"median_final_distance={float(np.median(distances)):.2f}m "
        f"mean_reward={float(np.mean(rewards)):.2f}"
    )


def _make_row(
    episode: int,
    step: int,
    t: float,
    info: dict,
    raw_action: np.ndarray,
    applied_action: np.ndarray,
    reward: float,
    total_reward: float,
    success: bool,
    crashed: bool,
    truncated: bool,
) -> dict:
    p = info["position"]
    v = info["velocity"]
    rpy = info["rpy"]
    goal = info["goal"]
    return {
        "episode": episode,
        "step": step,
        "time": t,
        "x": p[0],
        "y": p[1],
        "z": p[2],
        "vx": v[0],
        "vy": v[1],
        "vz": v[2],
        "roll": rpy[0],
        "pitch": rpy[1],
        "yaw": rpy[2],
        "goal_x": goal[0],
        "goal_y": goal[1],
        "goal_z": goal[2],
        "distance": info["distance"],
        "raw_action_vx": raw_action[0],
        "raw_action_vy": raw_action[1],
        "raw_action_vz": raw_action[2],
        "raw_action_yaw_rate": raw_action[3],
        "applied_action_vx": applied_action[0],
        "applied_action_vy": applied_action[1],
        "applied_action_vz": applied_action[2],
        "applied_action_yaw_rate": applied_action[3],
        "reward": reward,
        "total_reward": total_reward,
        "success": int(success),
        "crashed": int(crashed),
        "truncated": int(truncated),
    }


def _write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
