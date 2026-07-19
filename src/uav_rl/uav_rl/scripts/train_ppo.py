from __future__ import annotations

import argparse
from pathlib import Path

from uav_rl.envs import UavGoalEnv, UavGoalEnvConfig


TASK_GOALS = {
    "easy": ((-1.5, 1.5, 1.6), (1.5, 3.0, 2.1)),
    "medium": ((-3.0, 2.0, 1.5), (3.0, 5.0, 2.3)),
    "full": ((-5.0, 3.0, 1.5), (5.0, 8.0, 2.5)),
}


def main() -> None:
    parser = argparse.ArgumentParser(description="Train PPO on the UAV goal reaching environment.")
    # timesteps는 학습에 사용할 전체 환경 상호작용 횟수다.
    # num_envs는 여러 UAV 환경을 병렬로 돌려 PPO rollout 데이터를 더 빠르게 모으는 설정이다.
    parser.add_argument("--timesteps", type=int, default=200_000)
    parser.add_argument("--num-envs", type=int, default=8)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--out-dir", type=Path, default=Path("rl_runs/uav_goal_ppo"))
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--progress-bar", action="store_true")
    parser.add_argument("--task", choices=TASK_GOALS.keys(), default="medium")
    args = parser.parse_args()

    try:
        from stable_baselines3 import PPO
        from stable_baselines3.common.env_util import make_vec_env
        from stable_baselines3.common.callbacks import CheckpointCallback
    except ImportError as exc:
        raise SystemExit(
            "stable-baselines3 is not installed. Install RL dependencies first:\n"
            "  python3 -m pip install gymnasium stable-baselines3 tensorboard\n"
        ) from exc

    args.out_dir.mkdir(parents=True, exist_ok=True)

    # 각 UavGoalEnv는 프로젝트 구조를 축약해 포함한다:
    # 고수준 action -> 저수준 controller -> allocator/motor lag -> 6-DOF dynamics.
    goal_low, goal_high = TASK_GOALS[args.task]
    env_config = UavGoalEnvConfig(goal_low=goal_low, goal_high=goal_high)
    env = make_vec_env(lambda: UavGoalEnv(config=env_config), n_envs=args.num_envs, seed=args.seed)

    # observation이 이미지가 아니라 상태 벡터이므로 PyTorch 기반 MLP policy를 사용한다.
    # PPO는 환경마다 n_steps만큼 데이터를 모은 뒤, mini-batch를 n_epochs번 재사용해 학습한다.
    # clip_range는 policy가 한 번에 너무 크게 바뀌지 않도록 제한해 학습 안정성을 높인다.
    model = PPO(
        "MlpPolicy",  # 상태 벡터 입력이므로 CNN이 아닌 PyTorch MLP policy 사용.
        env,  # 병렬 UAV 학습 환경.
        learning_rate=3e-4,  # PPO에서 자주 쓰는 안정적인 기본 학습률.
        n_steps=1024,  # policy 업데이트 전 각 환경에서 모을 rollout 길이.
        batch_size=256,  # 수집한 rollout을 256개 단위 mini-batch로 나눠 학습.
        n_epochs=10,  # 같은 rollout 데이터를 10회 재사용해 sample efficiency 확보.
        gamma=0.99,  # 목표 도달처럼 미래 보상이 중요한 문제라 긴 시야를 둠.
        gae_lambda=0.95,  # advantage 계산에서 안정성과 긴 시야의 균형을 둔 값.
        clip_range=0.2,  # policy update 폭을 제한해 갑작스러운 행동 변화 방지.
        ent_coef=0.01,  # 초반 탐험을 유지하되 드론 제어가 너무 흔들리지 않게 작은 값 사용.
        verbose=1,  # 학습 진행 로그 출력.
        tensorboard_log=str(args.out_dir / "tb"),  # reward/loss curve 확인용 로그 저장.
        seed=args.seed,  # 실험 재현성을 위한 랜덤 seed.
        device=args.device,  # 기본은 CUDA. 작은 MLP는 CPU가 빠를 수도 있어 --device cpu로 비교 가능.
    )

    # 학습이 불안정해지거나 중간에 끊겨도 좋은 시점의 policy를 복구할 수 있게 저장한다.
    checkpoint = CheckpointCallback(
        save_freq=max(10_000 // max(args.num_envs, 1), 1),
        save_path=str(args.out_dir / "checkpoints"),
        name_prefix="uav_goal_ppo",
    )

    # learn()은 UAV 환경 rollout 수집과 PyTorch policy/value network 업데이트를 반복한다.
    model.learn(total_timesteps=args.timesteps, callback=checkpoint, progress_bar=args.progress_bar)

    # Stable-Baselines3는 학습된 PyTorch policy를 zip 파일로 저장한다.
    model.save(str(args.out_dir / "final_model"))
    print(f"Saved PPO policy to {args.out_dir / 'final_model.zip'}")


if __name__ == "__main__":
    main()
