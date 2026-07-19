from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path

import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot a saved UAV RL rollout CSV.")
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    data = _read_csv(args.csv)
    out_path = args.out if args.out is not None else args.csv.with_suffix(".png")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit("matplotlib is required: python3 -m pip install matplotlib") from exc

    t = data["time"]
    x = data["x"]
    y = data["y"]
    z = data["z"]
    goal = np.array([data["goal_x"][-1], data["goal_y"][-1], data["goal_z"][-1]])

    fig = plt.figure(figsize=(14, 9))
    fig.suptitle("UAV RL Rollout Visualization", fontsize=15, fontweight="bold")

    # 3D projection이 가능한 Matplotlib 환경이면 3D 궤적을 그리고, 아니면 XY plot으로 대체한다.
    try:
        ax3d = fig.add_subplot(2, 2, 1, projection="3d")
        ax3d.plot(x, y, z, label="UAV trajectory", linewidth=2.0)
        ax3d.scatter(x[0], y[0], z[0], marker="s", s=50, label="Start")
        ax3d.scatter(goal[0], goal[1], goal[2], marker="*", s=120, label="Goal")
        ax3d.set_xlabel("X [m]")
        ax3d.set_ylabel("Y [m]")
        ax3d.set_zlabel("Z [m]")
        ax3d.set_title("3D Trajectory")
        ax3d.legend(loc="best")
    except Exception:
        ax3d = fig.add_subplot(2, 2, 1)
        ax3d.plot(x, y, label="UAV trajectory", linewidth=2.0)
        ax3d.scatter(x[0], y[0], marker="s", s=50, label="Start")
        ax3d.scatter(goal[0], goal[1], marker="*", s=120, label="Goal")
        ax3d.set_xlabel("X [m]")
        ax3d.set_ylabel("Y [m]")
        ax3d.set_title("3D Unavailable - XY Trajectory")
        ax3d.axis("equal")
        ax3d.grid(True, alpha=0.3)
        ax3d.legend(loc="best")

    ax_xy = fig.add_subplot(2, 2, 2)
    ax_xy.plot(x, y, linewidth=2.0, label="Trajectory")
    ax_xy.scatter(x[0], y[0], marker="s", s=50, label="Start")
    ax_xy.scatter(goal[0], goal[1], marker="*", s=120, label="Goal")
    ax_xy.set_xlabel("X [m]")
    ax_xy.set_ylabel("Y [m]")
    ax_xy.set_title("XY Trajectory")
    ax_xy.axis("equal")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend(loc="best")

    ax_z = fig.add_subplot(2, 2, 3)
    ax_z.plot(t, z, label="Altitude Z [m]", linewidth=2.0)
    ax_z.plot(t, data["distance"], label="Distance to goal [m]", linewidth=2.0)
    ax_z.set_xlabel("Time [s]")
    ax_z.set_title("Altitude and Goal Distance")
    ax_z.grid(True, alpha=0.3)
    ax_z.legend(loc="best")

    ax_action = fig.add_subplot(2, 2, 4)
    ax_action.plot(t, data["applied_action_vx"], label="vx residual")
    ax_action.plot(t, data["applied_action_vy"], label="vy residual")
    ax_action.plot(t, data["applied_action_vz"], label="vz residual")
    ax_action.plot(t, data["applied_action_yaw_rate"], label="yaw-rate residual")
    ax_action.set_xlabel("Time [s]")
    ax_action.set_ylabel("Normalized action")
    ax_action.set_title("Applied PPO Residual Action")
    ax_action.grid(True, alpha=0.3)
    ax_action.legend(loc="best", fontsize=8)

    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
    fig.savefig(out_path, dpi=180)
    print(f"saved_plot={out_path}")


def _read_csv(path: Path) -> dict[str, np.ndarray]:
    rows: list[dict[str, str]] = []
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"empty rollout csv: {path}")

    numeric: dict[str, np.ndarray] = {}
    for key in rows[0].keys():
        numeric[key] = np.array([float(row[key]) for row in rows], dtype=np.float64)
    return numeric


if __name__ == "__main__":
    main()
