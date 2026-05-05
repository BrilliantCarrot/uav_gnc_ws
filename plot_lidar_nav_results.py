#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
사용법:
    cd ~/uav_gnc_ws
    python3 plot_lidar_nav_results.py --base-dir ~/uav_gnc_ws

창을 띄우지 않고 이미지/README 파일만 저장하려면:
    python3 plot_lidar_nav_results.py --base-dir ~/uav_gnc_ws --no-show

출력 폴더:
    ~/uav_gnc_ws/week8_lidar_nav_results/

필요 입력 파일:
    nav_gps_ekf.csv
    sim_gps_ekf.csv
    planning_path_gps_ekf.csv

    nav_lidar_aided_ekf.csv
    sim_lidar_aided_ekf.csv
    planning_path_lidar_aided_ekf.csv

    nav_lidar_aided_ukf.csv
    sim_lidar_aided_ukf.csv
    planning_path_lidar_aided_ukf.csv

    nav_lidar_init_imu_only_ekf_failed.csv
    sim_lidar_init_imu_only_ekf_failed.csv
    planning_path_lidar_init_imu_only_ekf_failed.csv

plot_lidar_nav_results.py

이 파일은 UAV GNC 고도화 프로젝트 Week 8의 LiDAR-aided navigation 실험 결과를
시각화하고, README에 넣기 좋은 요약 표를 자동 생성하기 위한 Python 스크립트임.

주요 목적은 GPS-denied 조건에서 LiDAR-derived pose correction을 EKF/UKF에 넣었을 때
항법 추정 성능과 경로 추종 성능이 어떻게 달라지는지 비교하는 것임.

시각화 내용:
    1. XY trajectory 비교
       - sim ground truth 경로
       - nav estimate 경로
       - 장애물 footprint
       - D* Lite keypoints

    2. Z trajectory 비교
       - sim z
       - nav z
       - target z

    3. 3D trajectory 비교
       - sim/nav 3D 경로
       - 장애물 cylinder
       - D* Lite keypoints

    4. Axis-wise CTE
       - x/y/z 방향 추종 오차
       - 각 축별 RMSE

    5. Localization error
       - ||p_nav - p_sim||
       - x/y/z 방향 localization error

비교하는 실험 케이스:
    - GPS EKF Baseline
    - LiDAR-aided EKF
    - LiDAR-aided UKF
    - LiDAR-init + IMU-only EKF

이 스크립트는 제어/항법 로직에는 영향을 주지 않고,
실험 후 생성된 CSV 파일을 읽어서 분석과 시각화만 수행함.
"""

import argparse
import glob
import os
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

try:
    import yaml
except ImportError:
    yaml = None


CASES = [
    {
        "key": "gps_ekf",
        "label": "GPS EKF Baseline",
        "filter": "EKF",
        "gps_update": "ON",
        "lidar_update": "OFF",
        "lidar_init_only": "OFF",
        "nav_candidates": ["nav_gps_ekf.csv"],
        "sim_candidates": ["sim_gps_ekf.csv"],
        "path_candidates": ["planning_path_gps_ekf.csv"],
        "note": "IMU + GPS correction baseline",
    },
    {
        "key": "lidar_aided_ekf",
        "label": "LiDAR-aided EKF",
        "filter": "EKF",
        "gps_update": "OFF",
        "lidar_update": "ON",
        "lidar_init_only": "OFF",
        "nav_candidates": ["nav_lidar_aided_ekf.csv"],
        "sim_candidates": ["sim_lidar_aided_ekf.csv"],
        "path_candidates": ["planning_path_lidar_aided_ekf.csv"],
        "note": "GPS-denied, LiDAR pose correction fused with EKF",
    },
    {
        "key": "lidar_aided_ukf",
        "label": "LiDAR-aided UKF",
        "filter": "UKF",
        "gps_update": "OFF",
        "lidar_update": "ON",
        "lidar_init_only": "OFF",
        "nav_candidates": ["nav_lidar_aided_ukf.csv"],
        "sim_candidates": ["sim_lidar_aided_ukf.csv"],
        "path_candidates": ["planning_path_lidar_aided_ukf.csv"],
        "note": "GPS-denied, LiDAR pose correction fused with UKF",
    },
    {
        "key": "lidar_init_imu_only_ekf",
        "label": "LiDAR-init + IMU-only EKF",
        "filter": "EKF",
        "gps_update": "OFF",
        "lidar_update": "OFF",
        "lidar_init_only": "ON",
        "nav_candidates": [
            "nav_lidar_init_imu_only_ekf_failed.csv",
            "nav_lidar_init_imu_only_ekf_failed..csv",
        ],
        "sim_candidates": ["sim_lidar_init_imu_only_ekf_failed.csv"],
        "path_candidates": ["planning_path_lidar_init_imu_only_ekf_failed.csv"],
        "note": "Initial pose only, then IMU prediction only",
    },
]


DEFAULT_OBSTACLES = {
    "x": [3.0, 0.0, -1.5],
    "y": [1.0, 5.0, 3.0],
    "r": [1.0, 1.0, 0.8],
    "height": 3.0,
    "base_z": 0.0,
    "fly_altitude": 2.0,
    "goal_x": -5.0,
    "goal_y": 7.0,
}


def resolve_csv(base_dir: Path, candidates, required=True):
    for name in candidates:
        path = base_dir / name
        if path.exists():
            return path

    for name in candidates:
        stem = Path(name).stem.replace("..", ".")
        pattern = str(base_dir / f"*{stem}*.csv")
        matches = sorted(glob.glob(pattern))
        if matches:
            return Path(matches[0])

    if required:
        raise FileNotFoundError(f"CSV file not found. candidates={candidates}, base_dir={base_dir}")
    return None


def find_config_file(base_dir: Path, filename: str):
    candidates = [
        base_dir / "src" / "uav_gnc" / "config" / filename,
        base_dir / filename,
        Path.cwd() / filename,
    ]
    for path in candidates:
        if path.exists():
            return path
    return None


def load_obstacles(base_dir: Path, planner_yaml=None, virtual_lidar_yaml=None):
    data = dict(DEFAULT_OBSTACLES)

    planner_path = Path(planner_yaml).expanduser() if planner_yaml else find_config_file(base_dir, "planner.yaml")
    lidar_path = Path(virtual_lidar_yaml).expanduser() if virtual_lidar_yaml else find_config_file(base_dir, "virtual_lidar.yaml")

    if yaml is not None and planner_path is not None and planner_path.exists():
        with open(planner_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
        params = cfg.get("path_planner_node", {}).get("ros__parameters", {})
        data["x"] = params.get("obstacle_x", data["x"])
        data["y"] = params.get("obstacle_y", data["y"])
        data["r"] = params.get("obstacle_r", data["r"])
        data["fly_altitude"] = float(params.get("fly_altitude", data["fly_altitude"]))
        data["goal_x"] = float(params.get("goal_x", data["goal_x"]))
        data["goal_y"] = float(params.get("goal_y", data["goal_y"]))

    if yaml is not None and lidar_path is not None and lidar_path.exists():
        with open(lidar_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
        params = cfg.get("virtual_lidar_node", {}).get("ros__parameters", {})
        data["height"] = float(params.get("obstacle_height", data["height"]))
        data["base_z"] = float(params.get("obstacle_base_z", data["base_z"]))

    data["x"] = [float(v) for v in data["x"]]
    data["y"] = [float(v) for v in data["y"]]
    data["r"] = [float(v) for v in data["r"]]
    return data


def load_case_data(base_dir: Path):
    loaded = []
    for case in CASES:
        nav_path = resolve_csv(base_dir, case["nav_candidates"], required=True)
        sim_path = resolve_csv(base_dir, case["sim_candidates"], required=True)
        plan_path = resolve_csv(base_dir, case["path_candidates"], required=False)

        df_nav = pd.read_csv(nav_path)
        df_sim = pd.read_csv(sim_path)
        if df_nav.empty or df_sim.empty:
            raise ValueError(f"Empty CSV detected: {nav_path} or {sim_path}")

        loaded_case = dict(case)
        loaded_case["nav_path"] = nav_path
        loaded_case["sim_path"] = sim_path
        loaded_case["plan_path"] = plan_path
        loaded_case["df_nav"] = df_nav
        loaded_case["df_sim"] = df_sim
        loaded_case["df_plan"] = pd.read_csv(plan_path) if plan_path is not None else None
        loaded.append(loaded_case)
    return loaded


def rmse(arr):
    arr = np.asarray(arr, dtype=float)
    if arr.size == 0:
        return np.nan
    return float(np.sqrt(np.mean(arr ** 2)))


def max_abs(arr):
    arr = np.asarray(arr, dtype=float)
    if arr.size == 0:
        return np.nan
    return float(np.max(np.abs(arr)))


def get_time(df):
    return df["t_sec"].to_numpy(dtype=float)


def get_xyz(df):
    return (
        df["x"].to_numpy(dtype=float),
        df["y"].to_numpy(dtype=float),
        df["z"].to_numpy(dtype=float),
    )


def select_planned_path(df_plan, method="longest"):
    if df_plan is None or df_plan.empty or "path_id" not in df_plan.columns:
        return None

    groups = []
    for path_id, g in df_plan.groupby("path_id"):
        g = g.sort_values("seq_idx")
        if len(g) < 2:
            continue
        groups.append((path_id, g))

    if not groups:
        return None

    if method == "last":
        _, chosen = groups[-1]
    elif method == "first":
        _, chosen = groups[0]
    else:
        _, chosen = max(groups, key=lambda item: (len(item[1]), -float(item[1]["recv_time_sec"].iloc[0]) if "recv_time_sec" in item[1].columns else 0.0))

    return (
        chosen["x"].to_numpy(dtype=float),
        chosen["y"].to_numpy(dtype=float),
        chosen["z"].to_numpy(dtype=float),
    )


def align_nav_to_sim(df_sim, df_nav):
    t_sim = get_time(df_sim)
    t_nav = get_time(df_nav)
    t_start = max(float(t_sim[0]), float(t_nav[0]))
    t_end = min(float(t_sim[-1]), float(t_nav[-1]))
    mask = (t_sim >= t_start) & (t_sim <= t_end)
    t_common = t_sim[mask]

    sim_x = df_sim["x"].to_numpy(dtype=float)[mask]
    sim_y = df_sim["y"].to_numpy(dtype=float)[mask]
    sim_z = df_sim["z"].to_numpy(dtype=float)[mask]
    nav_x = np.interp(t_common, t_nav, df_nav["x"].to_numpy(dtype=float))
    nav_y = np.interp(t_common, t_nav, df_nav["y"].to_numpy(dtype=float))
    nav_z = np.interp(t_common, t_nav, df_nav["z"].to_numpy(dtype=float))
    return t_common, sim_x, sim_y, sim_z, nav_x, nav_y, nav_z


def compute_case_metrics(case):
    df_nav = case["df_nav"]
    df_sim = case["df_sim"]
    t_common, sim_x, sim_y, sim_z, nav_x, nav_y, nav_z = align_nav_to_sim(df_sim, df_nav)

    err_x = nav_x - sim_x
    err_y = nav_y - sim_y
    err_z = nav_z - sim_z
    err_norm = np.sqrt(err_x ** 2 + err_y ** 2 + err_z ** 2)

    sim_completed = bool((df_sim["completed"].to_numpy(dtype=int) == 1).any()) if "completed" in df_sim.columns else False
    nav_completed = bool((df_nav["completed"].to_numpy(dtype=int) == 1).any()) if "completed" in df_nav.columns else False

    def completion_time(df):
        if "time_to_complete_sec" not in df.columns:
            return np.nan
        vals = df["time_to_complete_sec"].to_numpy(dtype=float)
        vals = vals[vals >= 0.0]
        return float(vals[0]) if vals.size > 0 else np.nan

    def goal_error_at_completion(df):
        if "completed" not in df.columns or "dist_to_wp" not in df.columns:
            return np.nan
        comp = df[df["completed"].astype(int) == 1]
        if comp.empty:
            return np.nan
        return float(comp["dist_to_wp"].iloc[0])

    metrics = {
        "case": case["label"],
        "filter": case["filter"],
        "gps_update": case["gps_update"],
        "lidar_update": case["lidar_update"],
        "lidar_init_only": case["lidar_init_only"],
        "mission_nav": nav_completed,
        "mission_sim": sim_completed,
        "time_nav": completion_time(df_nav),
        "time_sim": completion_time(df_sim),
        "tracking_rmse_nav": rmse(df_nav["cross_track_err"]) if "cross_track_err" in df_nav.columns else np.nan,
        "tracking_rmse_sim": rmse(df_sim["cross_track_err"]) if "cross_track_err" in df_sim.columns else np.nan,
        "tracking_max_nav": max_abs(df_nav["cross_track_err"]) if "cross_track_err" in df_nav.columns else np.nan,
        "tracking_max_sim": max_abs(df_sim["cross_track_err"]) if "cross_track_err" in df_sim.columns else np.nan,
        "loc_rmse_x": rmse(err_x),
        "loc_rmse_y": rmse(err_y),
        "loc_rmse_z": rmse(err_z),
        "loc_rmse_3d": rmse(err_norm),
        "loc_max_3d": max_abs(err_norm),
        "goal_error_at_completion_sim": goal_error_at_completion(df_sim),
        "min_goal_error_sim": float(df_sim["dist_to_wp"].min()) if "dist_to_wp" in df_sim.columns else np.nan,
        "final_goal_error_sim": float(df_sim["dist_to_wp"].iloc[-1]) if "dist_to_wp" in df_sim.columns else np.nan,
        "note": case["note"],
    }
    case["metrics"] = metrics
    case["aligned"] = {
        "t": t_common,
        "sim_x": sim_x,
        "sim_y": sim_y,
        "sim_z": sim_z,
        "nav_x": nav_x,
        "nav_y": nav_y,
        "nav_z": nav_z,
        "err_x": err_x,
        "err_y": err_y,
        "err_z": err_z,
        "err_norm": err_norm,
    }
    return metrics


def set_equal_3d_axes(ax, xs, ys, zs):
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    zs = np.asarray(zs, dtype=float)
    x_mid = 0.5 * (np.nanmin(xs) + np.nanmax(xs))
    y_mid = 0.5 * (np.nanmin(ys) + np.nanmax(ys))
    z_mid = 0.5 * (np.nanmin(zs) + np.nanmax(zs))
    max_range = max(np.nanmax(xs) - np.nanmin(xs), np.nanmax(ys) - np.nanmin(ys), np.nanmax(zs) - np.nanmin(zs), 1.0)
    half = 0.5 * max_range
    ax.set_xlim(x_mid - half, x_mid + half)
    ax.set_ylim(y_mid - half, y_mid + half)
    ax.set_zlim(max(0.0, z_mid - half), z_mid + half)


def add_obstacles_xy(ax, obstacles, label_once=True):
    for i, (ox, oy, rr) in enumerate(zip(obstacles["x"], obstacles["y"], obstacles["r"])):
        label = "Obstacle" if label_once and i == 0 else None
        circle = Circle((ox, oy), rr, fill=False, linewidth=1.5, linestyle="-", label=label)
        ax.add_patch(circle)
        ax.scatter([ox], [oy], marker="x", s=35)


def add_obstacles_3d(ax, obstacles, label_once=True):
    z0 = obstacles.get("base_z", 0.0)
    z1 = z0 + obstacles.get("height", 3.0)
    theta = np.linspace(0, 2 * np.pi, 36)
    z = np.linspace(z0, z1, 8)
    theta_grid, z_grid = np.meshgrid(theta, z)

    for i, (ox, oy, rr) in enumerate(zip(obstacles["x"], obstacles["y"], obstacles["r"])):
        x_grid = ox + rr * np.cos(theta_grid)
        y_grid = oy + rr * np.sin(theta_grid)
        ax.plot_surface(x_grid, y_grid, z_grid, alpha=0.18, linewidth=0, shade=True)
        ax.plot(ox + rr * np.cos(theta), oy + rr * np.sin(theta), np.full_like(theta, z0), linewidth=0.8)
        ax.plot(ox + rr * np.cos(theta), oy + rr * np.sin(theta), np.full_like(theta, z1), linewidth=0.8)
        if label_once and i == 0:
            ax.scatter([ox], [oy], [z1], marker="^", s=25, label="Obstacle")
        else:
            ax.scatter([ox], [oy], [z1], marker="^", s=25)


def get_goal_and_start(case, obstacles):
    df_sim = case["df_sim"]
    sim_x, sim_y, sim_z = get_xyz(df_sim)
    start = (float(sim_x[0]), float(sim_y[0]), float(sim_z[0]))
    if "wp_x" in df_sim.columns and "wp_y" in df_sim.columns and "wp_z" in df_sim.columns:
        goal = (float(df_sim["wp_x"].iloc[-1]), float(df_sim["wp_y"].iloc[-1]), float(df_sim["wp_z"].iloc[-1]))
    else:
        goal = (obstacles["goal_x"], obstacles["goal_y"], obstacles["fly_altitude"])
    return start, goal


def plot_planned_path_xy(ax, case):
    planned = select_planned_path(case.get("df_plan"), method="longest")
    if planned is None:
        return False
    px, py, _ = planned
    ax.plot(px, py, "o--", markersize=4, linewidth=1.0, label="D* Lite keypoints")
    return True


def plot_planned_path_3d(ax, case):
    planned = select_planned_path(case.get("df_plan"), method="longest")
    if planned is None:
        return False
    px, py, pz = planned
    ax.plot(px, py, pz, "o--", markersize=4, linewidth=1.0, label="D* Lite keypoints")
    return True


def plot_figure1_trajectory_xy_z(cases, output_dir: Path, obstacles):
    fig, axes = plt.subplots(len(cases), 2, figsize=(16, 4.0 * len(cases)))
    fig.suptitle("Week 8 UAV Navigation Result - XY and Z Trajectory", fontsize=15, fontweight="bold")

    for row, case in enumerate(cases):
        df_sim = case["df_sim"]
        df_nav = case["df_nav"]
        t_sim = get_time(df_sim)
        t_nav = get_time(df_nav)
        sim_x, sim_y, sim_z = get_xyz(df_sim)
        nav_x, nav_y, nav_z = get_xyz(df_nav)
        start, goal = get_goal_and_start(case, obstacles)

        ax_xy = axes[row, 0]
        ax_z = axes[row, 1]

        ax_xy.plot(sim_x, sim_y, label="Sim / Ground Truth", linewidth=1.8)
        ax_xy.plot(nav_x, nav_y, "--", label="Nav Estimate", linewidth=1.4)
        plot_planned_path_xy(ax_xy, case)
        add_obstacles_xy(ax_xy, obstacles)
        ax_xy.scatter(start[0], start[1], marker="s", s=50, label="Start")
        ax_xy.scatter(goal[0], goal[1], marker="*", s=90, label="Goal")
        ax_xy.set_title(f"{case['label']} - XY Trajectory")
        ax_xy.set_xlabel("X [m]")
        ax_xy.set_ylabel("Y [m]")
        ax_xy.axis("equal")
        ax_xy.grid(True, alpha=0.35)
        ax_xy.legend(fontsize=8, loc="best")

        ax_z.plot(t_sim, sim_z, label="Sim Z / Ground Truth", linewidth=1.8)
        ax_z.plot(t_nav, nav_z, "--", label="Nav Z Estimate", linewidth=1.4)
        target_z = goal[2]
        ax_z.axhline(target_z, label="Target Z", linewidth=1.2, alpha=0.8)
        ax_z.set_title(f"{case['label']} - Z Trajectory")
        ax_z.set_xlabel("Time [s]")
        ax_z.set_ylabel("Z [m]")
        ax_z.grid(True, alpha=0.35)
        ax_z.legend(fontsize=8, loc="best")

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    save_path = output_dir / "fig1_xy_z_trajectory.png"
    fig.savefig(save_path, dpi=200, bbox_inches="tight")
    print(f"[저장 완료] {save_path}")


def plot_figure2_3d_and_cte(cases, output_dir: Path, obstacles):
    fig = plt.figure(figsize=(18, 4.2 * len(cases)))
    fig.suptitle("Week 8 UAV Navigation Result - 3D Trajectory and Axis-wise CTE", fontsize=15, fontweight="bold")

    for row, case in enumerate(cases):
        df_sim = case["df_sim"]
        df_nav = case["df_nav"]
        t = get_time(df_sim)
        sim_x, sim_y, sim_z = get_xyz(df_sim)
        nav_x, nav_y, nav_z = get_xyz(df_nav)
        start, goal = get_goal_and_start(case, obstacles)

        ax3d = fig.add_subplot(len(cases), 2, row * 2 + 1, projection="3d")
        ax_cte = fig.add_subplot(len(cases), 2, row * 2 + 2)

        ax3d.plot(sim_x, sim_y, sim_z, label="Sim / Ground Truth", linewidth=1.6)
        ax3d.plot(nav_x, nav_y, nav_z, "--", label="Nav Estimate", linewidth=1.2)
        plotted_path = plot_planned_path_3d(ax3d, case)
        add_obstacles_3d(ax3d, obstacles)
        ax3d.scatter([start[0]], [start[1]], [start[2]], marker="s", s=30, label="Start")
        ax3d.scatter([goal[0]], [goal[1]], [goal[2]], marker="*", s=70, label="Goal")
        ax3d.set_title(f"{case['label']} - 3D Trajectory")
        ax3d.set_xlabel("X [m]", fontsize=8)
        ax3d.set_ylabel("Y [m]", fontsize=8)
        ax3d.set_zlabel("Z [m]", fontsize=8)
        ax3d.tick_params(labelsize=7)
        ax3d.legend(fontsize=7, loc="upper left")

        extra_x = np.array(obstacles["x"] + [start[0], goal[0]])
        extra_y = np.array(obstacles["y"] + [start[1], goal[1]])
        extra_z = np.array([obstacles["base_z"], obstacles["base_z"] + obstacles["height"], start[2], goal[2]])
        set_equal_3d_axes(
            ax3d,
            np.concatenate([sim_x, nav_x, extra_x]),
            np.concatenate([sim_y, nav_y, extra_y]),
            np.concatenate([sim_z, nav_z, extra_z]),
        )

        cte_x = np.abs(df_sim["cte_x"].to_numpy(dtype=float))
        cte_y = np.abs(df_sim["cte_y"].to_numpy(dtype=float))
        cte_z = np.abs(df_sim["cte_z"].to_numpy(dtype=float))
        rmse_x = rmse(cte_x)
        rmse_y = rmse(cte_y)
        rmse_z = rmse(cte_z)

        ax_cte.plot(t, cte_x, label=f"|CTE X|, RMSE={rmse_x:.3f}m", linewidth=1.2)
        ax_cte.plot(t, cte_y, label=f"|CTE Y|, RMSE={rmse_y:.3f}m", linewidth=1.2)
        ax_cte.plot(t, cte_z, label=f"|CTE Z|, RMSE={rmse_z:.3f}m", linewidth=1.2)
        ax_cte.axhline(rmse_x, linestyle="--", linewidth=0.9, alpha=0.6)
        ax_cte.axhline(rmse_y, linestyle="--", linewidth=0.9, alpha=0.6)
        ax_cte.axhline(rmse_z, linestyle="--", linewidth=0.9, alpha=0.6)
        ax_cte.set_title(f"{case['label']} - Axis-wise CTE")
        ax_cte.set_xlabel("Time [s]")
        ax_cte.set_ylabel("Absolute CTE [m]")
        ax_cte.grid(True, alpha=0.35)
        ax_cte.legend(fontsize=8, loc="best")

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    save_path = output_dir / "fig2_3d_trajectory_and_cte.png"
    fig.savefig(save_path, dpi=200, bbox_inches="tight")
    print(f"[저장 완료] {save_path}")


def plot_figure3_localization_error(cases, output_dir: Path):
    fig, axes = plt.subplots(len(cases), 2, figsize=(16, 4.0 * len(cases)))
    fig.suptitle("Week 8 UAV Navigation Result - Localization Error", fontsize=15, fontweight="bold")

    for row, case in enumerate(cases):
        data = case["aligned"]
        t = data["t"]
        err_x = data["err_x"]
        err_y = data["err_y"]
        err_z = data["err_z"]
        err_norm = data["err_norm"]
        rmse_norm = rmse(err_norm)
        max_norm = max_abs(err_norm)

        ax_norm = axes[row, 0]
        ax_axis = axes[row, 1]
        ax_norm.plot(t, err_norm, linewidth=1.5, label="||p_nav - p_sim||")
        ax_norm.axhline(rmse_norm, linestyle="--", linewidth=1.1, label=f"3D RMSE={rmse_norm:.3f}m")
        ax_norm.axhline(max_norm, linestyle=":", linewidth=1.1, label=f"Max={max_norm:.3f}m")
        ax_norm.set_title(f"{case['label']} - Localization Error Norm")
        ax_norm.set_xlabel("Time [s]")
        ax_norm.set_ylabel("Position Error [m]")
        ax_norm.grid(True, alpha=0.35)
        ax_norm.legend(fontsize=8, loc="best")

        ax_axis.plot(t, err_x, label=f"e_x, RMSE={rmse(err_x):.3f}m", linewidth=1.2)
        ax_axis.plot(t, err_y, label=f"e_y, RMSE={rmse(err_y):.3f}m", linewidth=1.2)
        ax_axis.plot(t, err_z, label=f"e_z, RMSE={rmse(err_z):.3f}m", linewidth=1.2)
        ax_axis.axhline(0.0, linestyle="--", linewidth=0.8)
        ax_axis.set_title(f"{case['label']} - Axis-wise Localization Error")
        ax_axis.set_xlabel("Time [s]")
        ax_axis.set_ylabel("Nav - Sim [m]")
        ax_axis.grid(True, alpha=0.35)
        ax_axis.legend(fontsize=8, loc="best")

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    save_path = output_dir / "fig3_localization_error.png"
    fig.savefig(save_path, dpi=200, bbox_inches="tight")
    print(f"[저장 완료] {save_path}")


def bool_to_text(value):
    return "Yes" if bool(value) else "No"


def fmt(value, digits=3):
    if value is None or (isinstance(value, float) and np.isnan(value)):
        return "-"
    return f"{float(value):.{digits}f}"


def generate_markdown_summary(cases, output_dir: Path):
    condition_rows = []
    result_rows = []
    for case in cases:
        m = case["metrics"]
        condition_rows.append([case["label"], case["filter"], case["gps_update"], case["lidar_update"], case["lidar_init_only"], case["note"]])
        result_rows.append([
            case["label"], bool_to_text(m["mission_sim"]), fmt(m["time_sim"]),
            fmt(m["tracking_rmse_sim"]), fmt(m["tracking_max_sim"]),
            fmt(m["loc_rmse_3d"]), fmt(m["loc_max_3d"]),
            fmt(m["goal_error_at_completion_sim"]), fmt(m["min_goal_error_sim"]), case["note"],
        ])

    def make_md_table(headers, rows):
        lines = ["| " + " | ".join(headers) + " |", "| " + " | ".join(["---"] * len(headers)) + " |"]
        for row in rows:
            lines.append("| " + " | ".join(str(v) for v in row) + " |")
        return "\n".join(lines)

    md = []
    md.append("# Week 8 LiDAR-aided Navigation Experiment Summary\n")
    md.append("## Experiment Conditions\n")
    md.append(make_md_table(["Case", "Filter", "GPS Update", "LiDAR Update", "LiDAR Init Only", "Description"], condition_rows))
    md.append("\n\n## Result Summary\n")
    md.append(make_md_table([
        "Case", "Mission Complete (Sim)", "Time to Complete [s]", "Tracking RMSE (Sim) [m]",
        "Tracking Max Error (Sim) [m]", "Localization RMSE 3D [m]", "Localization Max Error 3D [m]",
        "Goal Error at Completion [m]", "Minimum Goal Error [m]", "Notes",
    ], result_rows))
    md.append("\n\n## Notes\n")
    md.append("- Tracking RMSE is computed from `cross_track_err` in the sim tracking CSV. It includes the initial takeoff transient.\n")
    md.append("- Localization RMSE is computed from `nav - sim` after time synchronization by interpolation.\n")
    md.append("- D* Lite keypoints are plotted only when `planning_path_*.csv` files are available.\n")
    md.append("\n\n## Figures\n")
    md.append("- `fig1_xy_z_trajectory.png`: XY trajectory and Z trajectory comparison with obstacle footprints\n")
    md.append("- `fig2_3d_trajectory_and_cte.png`: 3D trajectory, obstacle cylinders, and axis-wise CTE\n")
    md.append("- `fig3_localization_error.png`: localization error norm and axis-wise localization error\n")
    md_text = "\n".join(md)
    save_path = output_dir / "week8_result_summary.md"
    save_path.write_text(md_text, encoding="utf-8")
    print(f"[저장 완료] {save_path}")
    return md_text


def print_terminal_summary(cases):
    print("\n========== Week 8 Result Summary ==========")
    for case in cases:
        m = case["metrics"]
        plan_status = "yes" if case.get("df_plan") is not None else "no"
        print(f"\n[{case['label']}]")
        print(f"  condition: filter={case['filter']}, GPS={case['gps_update']}, LiDAR={case['lidar_update']}, init_only={case['lidar_init_only']}")
        print(f"  planning_path_csv={plan_status}")
        print(f"  mission_sim={bool_to_text(m['mission_sim'])}, time_sim={fmt(m['time_sim'])} s")
        print(f"  tracking_rmse_sim={fmt(m['tracking_rmse_sim'])} m, tracking_max_sim={fmt(m['tracking_max_sim'])} m")
        print(f"  localization_rmse_3d={fmt(m['loc_rmse_3d'])} m, localization_max_3d={fmt(m['loc_max_3d'])} m")
        print(f"  loc_rmse_axis: x={fmt(m['loc_rmse_x'])}, y={fmt(m['loc_rmse_y'])}, z={fmt(m['loc_rmse_z'])} m")


def main():
    parser = argparse.ArgumentParser(description="Plot Week 8 UAV LiDAR-aided navigation results.")
    parser.add_argument("--base-dir", type=str, default=".", help="CSV 파일들이 있는 폴더. 예: /home/lyj/uav_gnc_ws")
    parser.add_argument("--output-dir", type=str, default=None, help="결과 이미지/markdown 저장 폴더")
    parser.add_argument("--planner-yaml", type=str, default=None, help="planner.yaml 경로")
    parser.add_argument("--virtual-lidar-yaml", type=str, default=None, help="virtual_lidar.yaml 경로")
    parser.add_argument("--no-show", action="store_true", help="그래프 창을 띄우지 않고 파일 저장만 함")
    args = parser.parse_args()

    base_dir = Path(args.base_dir).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve() if args.output_dir else base_dir / "week8_lidar_nav_results"
    output_dir.mkdir(parents=True, exist_ok=True)

    obstacles = load_obstacles(base_dir, args.planner_yaml, args.virtual_lidar_yaml)
    cases = load_case_data(base_dir)
    for case in cases:
        compute_case_metrics(case)

    print_terminal_summary(cases)
    plot_figure1_trajectory_xy_z(cases, output_dir, obstacles)
    plot_figure2_3d_and_cte(cases, output_dir, obstacles)
    plot_figure3_localization_error(cases, output_dir)
    generate_markdown_summary(cases, output_dir)

    if not args.no_show:
        plt.show()
    else:
        plt.close("all")


if __name__ == "__main__":
    main()
