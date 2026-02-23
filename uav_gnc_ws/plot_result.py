import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

def main():
    # 파일 경로 설정 (ROS2 launch 실행 위치에 따라 다를 수 있음)
    sim_file = 'sim_tracking_eval.csv'
    nav_file = 'nav_tracking_eval.csv'

    # === [추가할 부분] 저장할 폴더 경로 설정 및 생성 ===
    save_dir = '/home/lyj/simulation_result/images'
    os.makedirs(save_dir, exist_ok=True) # 폴더가 없으면 알아서 만들고, 있으면 무시함

    # 파일 존재 여부 확인 먼저 함
    if not os.path.exists(sim_file) or not os.path.exists(nav_file):
        print("CSV 파일이 없음. 드론 비행(launch)을 끝까지 완료하여 파일을 생성하거나, 파일을 현재 경로로 옮겨주세요!")
        return

    # Pandas로 데이터 불러옴
    df_sim = pd.read_csv(sim_file)
    df_nav = pd.read_csv(nav_file)

    # yaml 파일에 있는 전체 웨이포인트를 직접 하드코딩함(시작점 포함), 수정하면 여기도 따라 수정
    # 사각형 궤적 웨이포인트
    # wp_x = [0.0, 5.0, 5.0, 0.0, 0.0]
    # wp_y = [0.0, 0.0, 5.0, 5.0, 0.0]
    # 둥르런 모양(칠각형) 웨이포인트
    wp_x = [0.0, 4.0, 5.0, 5.0, 4.0, 1.0, 0.0, 0.0]
    wp_y = [0.0, 0.0, 1.0, 4.0, 5.0, 5.0, 4.0, 0.0]

    # ====== 1. 2D 궤적 (X-Y) 비교 그래프 ======
    plt.figure(figsize=(10, 8))
    
    # Waypoint 시각화 (목표 궤적)
    plt.plot(wp_x, wp_y, 'ro--', label='Waypoints (Target)', markersize=8, alpha=0.7)
    
    # 시뮬레이션 참값 (Ground Truth) 시각화
    plt.plot(df_sim['x'], df_sim['y'], 'b-', label='Sim (Ground Truth)', linewidth=2)
    
    # EKF 추정값 시각화
    plt.plot(df_nav['x'], df_nav['y'], 'g--', label='EKF (Estimated)', linewidth=2)

    plt.title('2D Trajectory Tracking Performance', fontsize=16)
    plt.xlabel('X Position (m)', fontsize=12)
    plt.ylabel('Y Position (m)', fontsize=12)
    plt.grid(True)
    plt.legend(fontsize=12)
    plt.axis('equal') # 비율 맞춤 (찌그러지지 않게)
    
    # === [수정할 부분] 경로를 합쳐서 저장 ===
    save_path_2d = os.path.join(save_dir, 'trajectory_2d.png')
    plt.savefig(save_path_2d, dpi=300)
    print(f"{save_path_2d} 궤적 이미지 저장 완료")

    # ====== 2. 시간에 따른 교차 항적 오차 (Cross-Track Error) ======
    plt.figure(figsize=(10, 5))
    
    # Sim 데이터 기준 오차
    plt.plot(df_sim['t_sec'], df_sim['cross_track_err'], 'b-', label='Sim CTE', alpha=0.8)
    # Nav 데이터 기준 오차
    plt.plot(df_nav['t_sec'], df_nav['cross_track_err'], 'g--', label='EKF CTE', alpha=0.8)

    plt.title('Cross-Track Error Over Time', fontsize=16)
    plt.xlabel('Time (sec)', fontsize=12)
    plt.ylabel('CTE (m)', fontsize=12)
    plt.grid(True)
    plt.legend(fontsize=12)
    
    # 오차가 0인 기준선 그림
    plt.axhline(0, color='red', linestyle=':', alpha=0.5)

    # === [수정할 부분] 경로를 합쳐서 저장 ===
    save_path_cte = os.path.join(save_dir, 'cte_over_time.png')
    plt.savefig(save_path_cte, dpi=300)
    print(f"{save_path_cte} 오차 이미지 저장 완료")

    # ====== 3. 성능 지표 (Metrics) 요약 ======
    # RMSE (Root Mean Square Error) 계산함
    sim_rmse = np.sqrt(np.mean(df_sim['cross_track_err']**2))
    nav_rmse = np.sqrt(np.mean(df_nav['cross_track_err']**2))
    
    sim_max_err = df_sim['cross_track_err'].abs().max()
    nav_max_err = df_nav['cross_track_err'].abs().max()

    print("\n" + "="*40)
    print("🚀 비행 성능 분석 결과 (Performance Metrics)")
    print("="*40)
    print(f"1. Ground Truth (Sim) 기준")
    print(f"   - RMSE: {sim_rmse:.4f} m")
    print(f"   - Max Error: {sim_max_err:.4f} m")
    print(f"2. EKF 추정 (Nav) 기준")
    print(f"   - RMSE: {nav_rmse:.4f} m")
    print(f"   - Max Error: {nav_max_err:.4f} m")
    print("="*40)

    # 그래프 화면에 띄움
    plt.show()

if __name__ == '__main__':
    main()