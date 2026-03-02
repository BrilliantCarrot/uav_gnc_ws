"""
rosbag에서 control_latency_ms 토픽을 읽어서 latency 그래프를 그리는 스크립트
사용법: python3 latency_plot.py <bag_path>
"""
import sys
import sqlite3
import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from amr_msgs.msg import ControlLatency


def plot_latency(bag_path: str):
    db_path = f'{bag_path}/{bag_path.split("/")[-1]}_0.db3'
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    cursor.execute(
        "SELECT id FROM topics WHERE name='/metrics/control_latency_ms'"
    )
    row = cursor.fetchone()
    if row is None:
        print('[ERROR] /metrics/control_latency_ms 토픽이 bag에 없음')
        return
    topic_id = row[0]

    cursor.execute(
        'SELECT data FROM messages WHERE topic_id=?', (topic_id,)
    )
    rows = cursor.fetchall()
    conn.close()

    latency_list = []
    for (data,) in rows:
        msg = deserialize_message(data, ControlLatency)
        latency_list.append(msg.latency_ms)

    arr = np.array(latency_list)
    print(f'평균 latency : {np.mean(arr):.2f} ms')
    print(f'99퍼센타일   : {np.percentile(arr, 99):.2f} ms')
    print(f'최대 latency : {np.max(arr):.2f} ms')

    # 그래프 출력
    plt.figure(figsize=(10, 4))
    plt.plot(arr, label='latency (ms)')
    plt.axhline(np.percentile(arr, 99), color='r',
                linestyle='--', label='99th percentile')
    plt.xlabel('샘플')
    plt.ylabel('ms')
    plt.title('Control Loop Latency')
    plt.legend()
    plt.tight_layout()
    plt.savefig(f'{bag_path}/latency_plot.png')
    plt.show()
    print(f'그래프 저장됨: {bag_path}/latency_plot.png')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('사용법: python3 latency_plot.py <bag_path>')
        sys.exit(1)
    plot_latency(sys.argv[1])