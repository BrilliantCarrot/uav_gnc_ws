"""
rosbag에서 pose_rmse 토픽을 읽어서 RMSE 통계를 출력하는 스크립트
사용법: python3 rmse_calc.py <bag_path>
"""
import sys
import sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from amr_msgs.msg import PoseRmse


def calc_rmse(bag_path: str):
    # sqlite3로 bag 파일 직접 읽기
    db_path = f'{bag_path}/{bag_path.split("/")[-1]}_0.db3'
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # 토픽 id 조회
    cursor.execute(
        "SELECT id FROM topics WHERE name='/metrics/pose_rmse'"
    )
    row = cursor.fetchone()
    if row is None:
        print('[ERROR] /metrics/pose_rmse 토픽이 bag에 없음')
        return
    topic_id = row[0]

    # 메시지 전체 읽기
    cursor.execute(
        'SELECT data FROM messages WHERE topic_id=?', (topic_id,)
    )
    rows = cursor.fetchall()
    conn.close()

    # 역직렬화 후 값 수집
    rmse_total_list = []
    for (data,) in rows:
        msg = deserialize_message(data, PoseRmse)
        rmse_total_list.append(msg.rmse_total)

    arr = np.array(rmse_total_list)
    print(f'샘플 수      : {len(arr)}')
    print(f'평균 RMSE    : {np.mean(arr):.4f} m')
    print(f'최대 RMSE    : {np.max(arr):.4f} m')
    print(f'표준편차     : {np.std(arr):.4f} m')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('사용법: python3 rmse_calc.py <bag_path>')
        sys.exit(1)
    calc_rmse(sys.argv[1])