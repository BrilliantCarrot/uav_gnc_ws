# uav_rl

`uav_rl` adds the first reinforcement-learning layer for the UAV GNC project.

The initial task is single-UAV goal reaching:

```text
PPO policy
-> normalized residual velocity / yaw-rate command
-> geometric guidance prior
-> low-level velocity/attitude controller
-> Quad-X allocator
-> motor first-order lag
-> 6-DOF dynamics
-> observation / reward
```

This is intentionally hierarchical. The RL policy learns high-level autonomous
flight decisions while the classical control and actuator model preserve the
project's existing GNC structure.

The default environment uses residual RL: a simple goal-directed guidance prior
generates a nominal velocity command, and PPO learns a bounded correction around
that command. This avoids forcing PPO to discover stable takeoff and goal-seeking
from pure random exploration.

## Smoke Test

```bash
colcon build --symlink-install --packages-select uav_rl
source install/setup.bash
ros2 run uav_rl check_uav_rl_env --episodes 3
```

## PPO Training

Install optional RL dependencies first:

```bash
python3 -m pip install -r src/uav_rl/requirements-rl.txt
```

If you do not need progress bars, the minimal set is:

```bash
python3 -m pip install gymnasium stable-baselines3 tensorboard
```

Train:

```bash
source install/setup.bash
ros2 run uav_rl train_uav_ppo --timesteps 200000 --num-envs 8
```

The default training device is `cuda`. If PyTorch cannot use CUDA, Stable-Baselines3
falls back to CPU. To force CPU:

```bash
ros2 run uav_rl train_uav_ppo --timesteps 200000 --num-envs 8 --device cpu
```

Task difficulty can be selected with `--task easy|medium|full`. The default is
`medium`; use `full` after the policy is stable.

Short pipeline check:

```bash
ros2 run uav_rl train_uav_ppo --timesteps 8192 --num-envs 1 --out-dir /tmp/uav_rl_smoke --device cpu
```

Evaluate:

```bash
ros2 run uav_rl rollout_uav_policy --model rl_runs/uav_goal_ppo/final_model.zip --episodes 10 --task medium
```

Save successful rollout CSV files and plot one:

```bash
ros2 run uav_rl rollout_uav_policy \
  --model rl_runs/uav_goal_ppo/final_model.zip \
  --episodes 50 \
  --task medium \
  --save-dir rl_runs/uav_goal_ppo/eval_rollouts \
  --max-saved-episodes 5

ros2 run uav_rl plot_rl_rollout \
  --csv rl_runs/uav_goal_ppo/eval_rollouts/success_episode_000.csv
```

Only successful episodes are saved by default. Use `--save-failures` when failure
trajectories are needed for debugging.

## Current Scope

Implemented:

- Gymnasium-style single-UAV goal-reaching environment
- 6-DOF dynamics port matching the C++ simulator structure
- F450-like Quad-X allocator and motor lag model
- Residual high-level velocity/yaw-rate action interface
- Reward terms for progress, distance, smoothness, attitude rate, effort, crash, and success
- PPO training and rollout scripts
- CSV rollout export and 3D/2D trajectory plotting

Planned:

- ROS2 policy deployment node
- PID/MPC/RL evaluation plots
- obstacle-aware observation
- multi-UAV environment
- MAPPO/MARL formation and collision-avoidance tasks
