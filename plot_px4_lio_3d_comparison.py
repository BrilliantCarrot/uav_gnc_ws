#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PX4 + FAST-LIO2 비교 실험 trajectory/RMSE 시각화 후처리 스크립트.

기본 사용법:
    cd ~/uav_gnc_ws
    python3 plot_px4_lio_3d_comparison.py

입력:
    eval/px4_lio_compare/gps_only_samples.csv
    eval/px4_lio_compare/gps_lio_samples.csv
    eval/px4_lio_compare/gps_denied_lio_baro_samples.csv

기본 출력:
    images/px4_lio_xy_trajectory_comparison.png
    images/px4_lio_3d_trajectory_comparison.png
    images/px4_lio_axis_error_grid.png

그림 구성:
    - 위에서 본 XY trajectory 비교
    - Guidance reference trajectory 1개
    - GPS only 실제 추종 궤적
    - GPS + LIO 실제 추종 궤적
    - GPS-denied LIO + barometer 실제 추종 궤적
    - 각 케이스별 X/Y/Z 축 시간 오차와 RMSE
"""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

# 일부 pip/system matplotlib 조합에서는 명시 import를 해야 3D projection이 등록된다.
try:
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
except Exception as exc:  # pragma: no cover - 실행 환경 의존 fallback
    raise SystemExit(
        "Matplotlib 3D projection을 불러오지 못했습니다. "
        "python3-matplotlib 또는 현재 venv의 matplotlib 설치를 확인하세요."
    ) from exc


CASES = [
    {
        "key": "gps_only",
        "label": "GPS only",
        "color": "#1f77b4",
        "linestyle": "-",
    },
    {
        "key": "gps_lio",
        "label": "GPS + FAST-LIO2",
        "color": "#ff7f0e",
        "linestyle": "-",
    },
    {
        "key": "gps_denied_lio_baro",
        "label": "GPS-denied LIO + baro",
        "color": "#2ca02c",
        "linestyle": "-",
    },
]


def read_samples(path: Path) -> dict[str, np.ndarray]:
    if not path.exists():
        raise FileNotFoundError(f"CSV not found: {path}")

    rows = []
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)

    if not rows:
        raise RuntimeError(f"CSV is empty: {path}")

    keys = rows[0].keys()
    data = {}
    for key in keys:
        data[key] = np.array([float(row[key]) for row in rows], dtype=float)
    return data


def load_cases(input_dir: Path) -> tuple[dict[str, np.ndarray], list[tuple[dict, dict[str, np.ndarray]]]]:
    loaded = []
    for case in CASES:
        csv_path = input_dir / f"{case['key']}_samples.csv"
        loaded.append((case, read_samples(csv_path)))

    # 세 케이스의 guidance reference는 같은 mission이어야 한다.
    # 가장 짧은 GPS-only 파일의 reference를 기준 guidance 궤적으로 사용한다.
    reference = loaded[0][1]
    return reference, loaded


def set_equal_3d_axes(ax, xs: np.ndarray, ys: np.ndarray, zs: np.ndarray) -> None:
    x_min, x_max = float(np.nanmin(xs)), float(np.nanmax(xs))
    y_min, y_max = float(np.nanmin(ys)), float(np.nanmax(ys))
    z_min, z_max = float(np.nanmin(zs)), float(np.nanmax(zs))

    x_mid = 0.5 * (x_min + x_max)
    y_mid = 0.5 * (y_min + y_max)
    z_mid = 0.5 * (z_min + z_max)
    radius = 0.5 * max(x_max - x_min, y_max - y_min, z_max - z_min, 1.0)

    ax.set_xlim(x_mid - radius, x_mid + radius)
    ax.set_ylim(y_mid - radius, y_mid + radius)
    ax.set_zlim(max(0.0, z_mid - radius), z_mid + radius)


def axis_equal_xy(ax, xs: np.ndarray, ys: np.ndarray) -> None:
    x_min, x_max = float(np.nanmin(xs)), float(np.nanmax(xs))
    y_min, y_max = float(np.nanmin(ys)), float(np.nanmax(ys))
    x_mid = 0.5 * (x_min + x_max)
    y_mid = 0.5 * (y_min + y_max)
    radius = 0.5 * max(x_max - x_min, y_max - y_min, 1.0)
    ax.set_xlim(x_mid - radius, x_mid + radius)
    ax.set_ylim(y_mid - radius, y_mid + radius)


def rmse(values: np.ndarray) -> float:
    if len(values) == 0:
        return float("nan")
    return float(np.sqrt(np.mean(values * values)))


def output_with_suffix(output_path: Path, suffix: str) -> Path:
    if output_path.name == "px4_lio_3d_trajectory_comparison.png":
        return output_path.with_name(f"px4_lio_{suffix}.png")
    return output_path.with_name(f"{output_path.stem}_{suffix}.png")


def plot_xy_comparison(
    reference: dict[str, np.ndarray],
    loaded_cases: list[tuple[dict, dict[str, np.ndarray]]],
    output_path: Path,
    start_time_sec: float,
) -> None:
    fig, ax = plt.subplots(figsize=(10.0, 8.0))

    ref_mask = reference["t_sec"] >= start_time_sec
    ref_x = reference["ref_x"][ref_mask]
    ref_y = reference["ref_y"][ref_mask]

    ax.plot(
        ref_x,
        ref_y,
        color="black",
        linestyle="--",
        linewidth=2.2,
        label="Guidance reference",
    )

    all_x = [ref_x]
    all_y = [ref_y]

    for case, data in loaded_cases:
        mask = data["t_sec"] >= start_time_sec
        x = data["x"][mask]
        y = data["y"][mask]
        ax.plot(
            x,
            y,
            color=case["color"],
            linestyle=case["linestyle"],
            linewidth=1.8,
            label=case["label"],
        )
        all_x.append(x)
        all_y.append(y)

    if len(ref_x) > 0:
        ax.scatter([ref_x[0]], [ref_y[0]], marker="s", s=45, color="red", label="Start")
        ax.scatter([ref_x[-1]], [ref_y[-1]], marker="*", s=120, color="purple", label="Goal")

    ax.set_title("PX4 EKF2 GPS/LIO Fusion - XY Trajectory Comparison", fontsize=15, fontweight="bold")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.grid(True, alpha=0.35)
    ax.legend(fontsize=9, loc="best")
    axis_equal_xy(ax, np.concatenate(all_x), np.concatenate(all_y))

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=220, bbox_inches="tight")
    print(f"saved: {output_path}")


def plot_3d_comparison(
    reference: dict[str, np.ndarray],
    loaded_cases: list[tuple[dict, dict[str, np.ndarray]]],
    output_path: Path,
    title: str,
    start_time_sec: float,
) -> None:
    fig = plt.figure(figsize=(10.5, 8.0))
    ax = fig.add_subplot(111, projection="3d")

    ref_mask = reference["t_sec"] >= start_time_sec
    ref_x = reference["ref_x"][ref_mask]
    ref_y = reference["ref_y"][ref_mask]
    ref_z = reference["ref_z"][ref_mask]

    ax.plot(
        ref_x,
        ref_y,
        ref_z,
        color="black",
        linestyle="--",
        linewidth=2.2,
        label="Guidance reference",
    )

    all_x = [ref_x]
    all_y = [ref_y]
    all_z = [ref_z]

    for case, data in loaded_cases:
        mask = data["t_sec"] >= start_time_sec
        x = data["x"][mask]
        y = data["y"][mask]
        z = data["z"][mask]

        ax.plot(
            x,
            y,
            z,
            color=case["color"],
            linestyle=case["linestyle"],
            linewidth=1.8,
            label=case["label"],
        )

        all_x.append(x)
        all_y.append(y)
        all_z.append(z)

    if len(ref_x) > 0:
        ax.scatter([ref_x[0]], [ref_y[0]], [ref_z[0]], marker="s", s=45, color="red", label="Start")
        ax.scatter([ref_x[-1]], [ref_y[-1]], [ref_z[-1]], marker="*", s=120, color="purple", label="Goal")

    ax.set_title(title, fontsize=15, fontweight="bold", pad=18)
    ax.set_xlabel("X [m]", fontsize=10)
    ax.set_ylabel("Y [m]", fontsize=10)
    ax.set_zlabel("Z [m]", fontsize=10)
    ax.grid(True, alpha=0.35)
    ax.legend(fontsize=9, loc="upper left")
    ax.view_init(elev=26, azim=-58)

    set_equal_3d_axes(
        ax,
        np.concatenate(all_x),
        np.concatenate(all_y),
        np.concatenate(all_z),
    )

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=220, bbox_inches="tight")
    print(f"saved: {output_path}")


def plot_axis_error_grid(
    loaded_cases: list[tuple[dict, dict[str, np.ndarray]]],
    output_path: Path,
    start_time_sec: float,
) -> None:
    axes_info = [
        ("X", "err_x"),
        ("Y", "err_y"),
        ("Z", "err_z"),
    ]
    fig, axs = plt.subplots(len(loaded_cases), 3, figsize=(16.0, 3.6 * len(loaded_cases)), sharex=False)
    if len(loaded_cases) == 1:
        axs = np.array([axs])

    fig.suptitle("PX4 EKF2 GPS/LIO Fusion - Axis-wise Tracking Error", fontsize=15, fontweight="bold")

    for row, (case, data) in enumerate(loaded_cases):
        mask = data["t_sec"] >= start_time_sec
        t = data["t_sec"][mask]
        if len(t) > 0:
            t = t - t[0]

        for col, (axis_name, key) in enumerate(axes_info):
            ax = axs[row, col]
            err = np.abs(data[key][mask])
            err_rmse = rmse(data[key][mask])

            ax.plot(t, err, color=case["color"], linewidth=1.1)
            ax.axhline(err_rmse, color="black", linestyle="--", linewidth=1.0, label=f"RMSE={err_rmse:.3f}m")
            ax.set_title(f"{case['label']} - |Error {axis_name}|", fontsize=10, fontweight="bold")
            ax.set_xlabel("Time [s]")
            ax.set_ylabel(f"|Error {axis_name}| [m]")
            ax.grid(True, alpha=0.35)
            ax.legend(fontsize=8, loc="upper right")

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=220, bbox_inches="tight")
    print(f"saved: {output_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot PX4 GPS/LIO trajectory and RMSE comparison.")
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=Path("eval/px4_lio_compare"),
        help="Directory containing *_samples.csv files.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("images/px4_lio_3d_trajectory_comparison.png"),
        help="3D output PNG path. XY/error figures are saved next to this file.",
    )
    parser.add_argument(
        "--start-time-sec",
        type=float,
        default=0.0,
        help="Ignore samples before this time. Use 10.0 to remove takeoff transient.",
    )
    parser.add_argument(
        "--title",
        type=str,
        default="PX4 EKF2 GPS/LIO Fusion - 3D Trajectory Comparison",
        help="Figure title.",
    )
    args = parser.parse_args()

    reference, loaded_cases = load_cases(args.input_dir)
    plot_xy_comparison(
        reference,
        loaded_cases,
        output_with_suffix(args.output, "xy_trajectory_comparison"),
        args.start_time_sec,
    )
    plot_3d_comparison(reference, loaded_cases, args.output, args.title, args.start_time_sec)
    plot_axis_error_grid(
        loaded_cases,
        output_with_suffix(args.output, "axis_error_grid"),
        args.start_time_sec,
    )


if __name__ == "__main__":
    main()
