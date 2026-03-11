import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import os

def analyze_and_plot(csv_file, title_prefix, save_dir):
    # CSV 파일이 폴더에 정상적으로 생성되었는지 확인검사함
    if not os.path.exists(csv_file):
        print(f"[{csv_file}] 파일이 존재하지 않음. 시뮬레이션을 끝까지 돌렸는지 확인 요망!")
        return

    # pandas로 CSV 데이터 한 번에 읽어옴
    df = pd.read_csv(csv_file)

    if df.empty:
        print(f"[{csv_file}] 파일이 비어 있음! 데이터가 로깅되지 않음.")
        return

    # 데이터 추출함
    t = df['t_sec'].values
    x = df['x'].values
    y = df['y'].values
    z = df['z'].values 
    
    # === [수정됨] 1. 3D 공간 상의 웨이포인트 추출 (누락 및 중복 삭제 방지) ===
    # 시작점(wp[0])은 드론의 첫 위치로 근사하여 수동으로 넣어줌
    wp_x_list = [x[0]]  
    wp_y_list = [y[0]]
    wp_z_list = [z[0]]
    
    # 각 구간(seg_idx)이 타겟으로 삼고 있는 다음 웨이포인트들 (wp[1] ~ wp[last])을 순서대로 가져옴
    target_wps = df.groupby('seg_idx').first()[['wp_x', 'wp_y', 'wp_z']]
    wp_x_list.extend(target_wps['wp_x'].tolist())
    wp_y_list.extend(target_wps['wp_y'].tolist())
    wp_z_list.extend(target_wps['wp_z'].tolist())
    
    # === [수정됨] 2. 시간에 따른 고도(Z) 웨이포인트 도달 시점 추출 ===
    wp_times = [t[0]]       # 시작 시간에 0번 웨이포인트
    wp_z_timeseries = [z[0]]

    # 구간이 바뀌는 순간 = 이전 타겟 웨이포인트에 도달한 순간을 찾아냄
    seg_changes = df[df['seg_idx'].diff() > 0]
    for idx, row in seg_changes.iterrows():
        wp_times.append(row['t_sec'])
        # 구간이 바뀌기 직전(idx-1)의 목표 Z값이 우리가 방금 도달한 웨이포인트의 고도임
        wp_z_timeseries.append(df.loc[idx - 1, 'wp_z'])
        
    # 마지막 도착점 도달 순간 (completed == 1이 된 가장 첫 순간)
    completed_rows = df[df['completed'] == 1]
    if not completed_rows.empty:
        last_wp_time = completed_rows.iloc[0]['t_sec']
        last_wp_z = completed_rows.iloc[0]['wp_z']
        # 중복 추가 방지
        if wp_times[-1] != last_wp_time:
            wp_times.append(last_wp_time)
            wp_z_timeseries.append(last_wp_z)
    # =========================================================

    # 전체 오차 및 추가된 축별 에러 데이터 추출함
    cte = df['cross_track_err'].values
    cte_x = df['cte_x'].values
    cte_y = df['cte_y'].values
    cte_z = df['cte_z'].values

    # 평가 지표(RMSE, Max Error) 전체 및 축별 계산함
    rmse = np.sqrt(np.mean(cte**2))
    max_err = np.max(np.abs(cte))
    rmse_x = np.sqrt(np.mean(cte_x**2)); max_x = np.max(np.abs(cte_x))
    rmse_y = np.sqrt(np.mean(cte_y**2)); max_y = np.max(np.abs(cte_y))
    rmse_z = np.sqrt(np.mean(cte_z**2)); max_z = np.max(np.abs(cte_z))

    # 터미널에 전체 및 축별 상세 결과 출력함
    print(f"========== {title_prefix} ==========")
    print(f"[전체] RMSE : {rmse:.4f} m | Max Error : {max_err:.4f} m")
    print(f"[X축]  RMSE : {rmse_x:.4f} m | Max Error : {max_x:.4f} m")
    print(f"[Y축]  RMSE : {rmse_y:.4f} m | Max Error : {max_y:.4f} m")
    print(f"[Z축]  RMSE : {rmse_z:.4f} m | Max Error : {max_z:.4f} m")

    # 그래프 그리기 세팅함 (가로로 길게 1x3 배열로 확장함)
    fig = plt.figure(figsize=(18, 5))

    # 1. 3D Trajectory Plot (3차원 공간 시각화)
    ax1 = fig.add_subplot(1, 3, 1, projection='3d')
    ax1.plot(x, y, z, label='Actual Path', color='blue', linewidth=2)
    
    # 수정된 리스트로 3D 웨이포인트 정확하게 찍어줌
    ax1.plot(wp_x_list, wp_y_list, wp_z_list, 
             marker='o', color='red', linestyle='--', markersize=6, label='Waypoints')
    
    ax1.set_title(f"{title_prefix}\n3D Trajectory")
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_zlabel('Z [m]')
    ax1.legend()

    # 2. Altitude Profile (측면 고도 뷰)
    ax2 = fig.add_subplot(1, 3, 2)
    ax2.plot(t, z, label='Actual Altitude (Z)', color='blue', linewidth=2)
    
    # 수정된 타임라인으로 웨이포인트 도달 시점에 정확히 별표 찍음
    ax2.scatter(wp_times, wp_z_timeseries, color='red', marker='*', s=150, zorder=5, label='Waypoints Z')
    # 다음 웨이포인트를 향한 스텝은 그대로 옅게 그려줌
    ax2.step(t, df['wp_z'].values, where='post', color='red', linestyle='--', alpha=0.3, label='Upcoming Target Z')
    
    ax2.set_title(f"{title_prefix}\nAltitude Profile Over Time")
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Altitude (Z) [m]')
    ax2.grid(True)
    ax2.legend()

    # 3. Cross Track Error Plot (오차 추이)
    ax3 = fig.add_subplot(1, 3, 3)
    ax3.plot(t, cte, color='red', label='Total Error')
    # Z축 오차만 따로 볼 수 있도록 점선으로 겹쳐서 그림
    ax3.plot(t, np.abs(cte_z), color='green', linestyle=':', label='Z-axis Error', alpha=0.7)
    
    ax3.axhline(rmse, color='blue', linestyle='--', label=f'Total RMSE ({rmse:.4f}m)')
    ax3.set_title(f"{title_prefix}\nCross Track Error")
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Error [m]')
    ax3.grid(True)
    ax3.legend()

    plt.tight_layout()

    # 저장할 디렉토리가 없으면 생성함
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
        
    # 파일명을 파일 시스템에 맞게 안전하게 변경함
    safe_title = title_prefix.replace(' ', '_').replace('(', '').replace(')', '')
    save_path_3d = os.path.join(save_dir, f'trajectory_3d_{safe_title}.png')
    
    # 그래프 저장함
    plt.savefig(save_path_3d, dpi=300)

if __name__ == '__main__':
    save_directory = '/home/lyj/uav_gnc_ws'
    analyze_and_plot('sim_tracking_eval.csv', 'Simulation Only (Ideal)', save_directory)
    analyze_and_plot('nav_tracking_eval.csv', 'Navigation (EKF with Noise)', save_directory)
    plt.show()
