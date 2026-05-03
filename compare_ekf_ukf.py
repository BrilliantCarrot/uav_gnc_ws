"""
compare_ekf_ukf.py
==================
EKF vs UKF 성능 비교 분석 스크립트

사용법:
    python3 compare_ekf_ukf.py

필요 파일 (~/uav_gnc_ws/ 기준):
    ekf_nav.csv  ekf_sim.csv   ← EKF 실험 결과
    ukf_nav.csv  ukf_sim.csv   ← UKF 실험 결과

출력:
    ekf_ukf_comparison.png  ← 비교 그래프 (4개 서브플롯)
    콘솔에 수치 요약 테이블
"""

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ── 파일 경로 설정 ────────────────────────────────────────────
BASE = os.path.expanduser("~/uav_gnc_ws")

FILES = {
    "ekf": {
        "nav": os.path.join(BASE, "ekf_nav.csv"),
        "sim": os.path.join(BASE, "ekf_sim.csv"),
    },
    "ukf": {
        "nav": os.path.join(BASE, "ukf_nav.csv"),
        "sim": os.path.join(BASE, "ukf_sim.csv"),
    },
}

# ── 색상 ──────────────────────────────────────────────────────
COLOR = {"ekf": "#E07B54", "ukf": "#5B9BD5"}

# ═══════════════════════════════════════════════════════════════
# 헬퍼 함수
# ═══════════════════════════════════════════════════════════════

def load(path: str) -> pd.DataFrame:
    """CSV 로드 후 기본 검증"""
    if not os.path.exists(path):
        raise FileNotFoundError(f"파일 없음: {path}\n"
                                "실험 순서:\n"
                                "  1) EKF 실행 후 nav/sim → ekf_nav.csv / ekf_sim.csv\n"
                                "  2) UKF 실행 후 nav/sim → ukf_nav.csv / ukf_sim.csv")
    df = pd.read_csv(path)
    df.sort_values("t_sec", inplace=True)
    df.reset_index(drop=True, inplace=True)
    return df


def merge_sim_nav(sim: pd.DataFrame, nav: pd.DataFrame,
                  tolerance: float = 0.02) -> pd.DataFrame:
    """sim(실제)과 nav(추정) 시간 기준 병합 → 필터 추정 오차 계산"""
    merged = pd.merge_asof(
        sim[["t_sec", "x", "y", "z"]],
        nav[["t_sec", "x", "y", "z"]],
        on="t_sec", suffixes=("_sim", "_nav"),
        tolerance=tolerance,
    )
    merged.dropna(inplace=True)
    merged["err_x"]  = merged["x_sim"] - merged["x_nav"]
    merged["err_y"]  = merged["y_sim"] - merged["y_nav"]
    merged["err_z"]  = merged["z_sim"] - merged["z_nav"]
    merged["err_3d"] = np.sqrt(merged["err_x"]**2 +
                               merged["err_y"]**2 +
                               merged["err_z"]**2)
    return merged


def rmse(series: pd.Series) -> float:
    return float(np.sqrt((series**2).mean()))


def compute_metrics(sim: pd.DataFrame, nav: pd.DataFrame,
                    merged: pd.DataFrame) -> dict:
    """핵심 지표 딕셔너리 반환"""
    completed    = int(nav["completed"].max())
    mission_time = float(nav["time_to_complete_sec"].max()) if completed else None

    return {
        "completed"    : completed,
        "mission_time" : mission_time,
        "rmse_x"       : rmse(merged["err_x"]),
        "rmse_y"       : rmse(merged["err_y"]),
        "rmse_z"       : rmse(merged["err_z"]),
        "rmse_3d"      : rmse(merged["err_3d"]),
        "max_err_3d"   : float(merged["err_3d"].max()),
        "cte_rmse_sim" : rmse(sim["cross_track_err"]),   # 실제 궤적 추종 오차
        "cte_rmse_nav" : rmse(nav["cross_track_err"]),   # 추정 기반 궤적 추종 오차
    }


# ═══════════════════════════════════════════════════════════════
# 메인 분석
# ═══════════════════════════════════════════════════════════════

def main():
    # ── 데이터 로드 ─────────────────────────────────────────────
    data = {}
    for name, paths in FILES.items():
        sim    = load(paths["sim"])
        nav    = load(paths["nav"])
        merged = merge_sim_nav(sim, nav)
        data[name] = {
            "sim":     sim,
            "nav":     nav,
            "merged":  merged,
            "metrics": compute_metrics(sim, nav, merged),
        }

    ekf_m = data["ekf"]["metrics"]
    ukf_m = data["ukf"]["metrics"]

    # ── 콘솔 요약 출력 ──────────────────────────────────────────
    print("=" * 60)
    print("         EKF vs UKF 성능 비교 요약")
    print("=" * 60)
    labels = [
        ("미션 완료 여부",         "completed",    lambda v: "✓" if v else "✗"),
        ("미션 완료 시간 (s)",      "mission_time", lambda v: f"{v:.2f}" if v else "미완료"),
        ("필터 RMSE X (m)",        "rmse_x",       lambda v: f"{v:.4f}"),
        ("필터 RMSE Y (m)",        "rmse_y",       lambda v: f"{v:.4f}"),
        ("필터 RMSE Z (m)",        "rmse_z",       lambda v: f"{v:.4f}"),
        ("필터 RMSE 3D (m)",       "rmse_3d",      lambda v: f"{v:.4f}"),
        ("최대 3D 오차 (m)",       "max_err_3d",   lambda v: f"{v:.4f}"),
        ("궤적 추종 오차 RMSE (m)", "cte_rmse_sim", lambda v: f"{v:.4f}"),
    ]
    print(f"  {'항목':<25}{'EKF':>12}{'UKF':>12}{'개선율':>10}")
    print("-" * 60)
    for label, key, fmt in labels:
        e_val = ekf_m[key]
        u_val = ukf_m[key]
        e_str = fmt(e_val)
        u_str = fmt(u_val)
        # 수치 항목만 개선율 계산 (낮을수록 좋은 지표)
        try:
            if e_val and u_val and isinstance(e_val, float):
                improve = (e_val - u_val) / e_val * 100
                imp_str = f"{improve:+.1f}%"
            else:
                imp_str = ""
        except Exception:
            imp_str = ""
        print(f"  {label:<25}{e_str:>12}{u_str:>12}{imp_str:>10}")
    print("=" * 60)
    print("  개선율: + 값이면 UKF가 더 좋음, - 값이면 EKF가 더 좋음")
    print()

    # ── 그래프 ──────────────────────────────────────────────────
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle("EKF vs UKF Performance Comparison", fontsize=16, fontweight="bold")
    gs  = gridspec.GridSpec(2, 2, hspace=0.38, wspace=0.32)

    # ── Plot 1: 3D 필터 추정 오차 시계열 ───────────────────────
    ax1 = fig.add_subplot(gs[0, 0])
    for name in ("ekf", "ukf"):
        mg = data[name]["merged"]
        ax1.plot(mg["t_sec"], mg["err_3d"],
                 color=COLOR[name], linewidth=1.0,
                 label=f"{name.upper()} (RMSE={data[name]['metrics']['rmse_3d']:.3f}m)")
    ax1.set_title("Filter Estimation Error (sim vs nav)")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("3D Position Error (m)")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # ── Plot 2: Cross-Track Error 시계열 ───────────────────────
    ax2 = fig.add_subplot(gs[0, 1])
    for name in ("ekf", "ukf"):
        sim = data[name]["sim"]
        ax2.plot(sim["t_sec"], sim["cross_track_err"].abs(),
                 color=COLOR[name], linewidth=1.0,
                 label=f"{name.upper()} (RMSE={data[name]['metrics']['cte_rmse_sim']:.3f}m)")
    ax2.set_title("Cross-Track Error (Ground Truth)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("|Cross-Track Error| (m)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # ── Plot 3: XY 궤적 비교 ────────────────────────────────────
    ax3 = fig.add_subplot(gs[1, 0])
    # 웨이포인트 표시 (sim에서 추출)
    sim_ref = data["ekf"]["sim"]
    wps = sim_ref.drop_duplicates("seg_idx")[["wp_x", "wp_y"]].values
    ax3.plot(wps[:, 0], wps[:, 1], "k--", linewidth=1.0,
             alpha=0.4, label="Waypoints")
    ax3.scatter(wps[:, 0], wps[:, 1], c="k", s=30, zorder=5)

    for name in ("ekf", "ukf"):
        sim = data[name]["sim"]
        nav = data[name]["nav"]
        ax3.plot(sim["x"], sim["y"],
                 color=COLOR[name], linewidth=1.2, label=f"{name.upper()} sim")
        ax3.plot(nav["x"], nav["y"],
                 color=COLOR[name], linewidth=1.2, linestyle=":",
                 alpha=0.7, label=f"{name.upper()} nav")
    ax3.set_title("XY Trajectory")
    ax3.set_xlabel("X (m)")
    ax3.set_ylabel("Y (m)")
    ax3.legend(fontsize=7)
    ax3.set_aspect("equal")
    ax3.grid(True, alpha=0.3)

    # ── Plot 4: RMSE 막대 비교 ───────────────────────────────────
    ax4 = fig.add_subplot(gs[1, 1])
    metrics_keys  = ["rmse_x", "rmse_y", "rmse_z", "rmse_3d", "cte_rmse_sim"]
    metrics_names = ["RMSE X", "RMSE Y", "RMSE Z", "RMSE 3D", "CTE RMSE"]
    x_pos = np.arange(len(metrics_keys))
    width = 0.35

    ekf_vals = [ekf_m[k] for k in metrics_keys]
    ukf_vals = [ukf_m[k] for k in metrics_keys]

    bars_ekf = ax4.bar(x_pos - width/2, ekf_vals, width,
                       color=COLOR["ekf"], label="EKF", alpha=0.85)
    bars_ukf = ax4.bar(x_pos + width/2, ukf_vals, width,
                       color=COLOR["ukf"], label="UKF", alpha=0.85)

    # 막대 위 수치 표기
    for bar in bars_ekf:
        ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.005,
                 f"{bar.get_height():.3f}", ha="center", va="bottom", fontsize=7)
    for bar in bars_ukf:
        ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.005,
                 f"{bar.get_height():.3f}", ha="center", va="bottom", fontsize=7)

    ax4.set_title("RMSE Comparison (m)")
    ax4.set_xticks(x_pos)
    ax4.set_xticklabels(metrics_names, rotation=15, ha="right")
    ax4.set_ylabel("RMSE (m)")
    ax4.legend()
    ax4.grid(True, alpha=0.3, axis="y")

    # ── 저장 ────────────────────────────────────────────────────
    out_path = os.path.join(BASE, "ekf_ukf_comparison.png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"그래프 저장 완료: {out_path}")
    plt.show()


if __name__ == "__main__":
    main()