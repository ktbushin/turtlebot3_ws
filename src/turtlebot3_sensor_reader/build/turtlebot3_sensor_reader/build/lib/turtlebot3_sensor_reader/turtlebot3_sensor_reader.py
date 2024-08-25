import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import json

class SensorReader(Node):
    def __init__(self, max_scans=3):
        super().__init__('sensor_reader')
                
        self.max_scans = max_scans  # 最大スキャン回数を設定
        self.scan_count = 0  # スキャン回数をカウントする変数

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # データをため込むリスト
        self.data_storage = []

        # JSONファイルの準備
        self.json_file_path = 'config/sensor_data.json'

    def listener_callback(self, msg):
        # センサー値を取得してリストに追加
        ranges = list(msg.ranges)  # arrayをリストに変換
        timestamp = self.get_clock().now().to_msg().sec  # タイムスタンプを取得
        self.data_storage.append({'timestamp': timestamp, 'ranges': ranges})

        # スキャン回数をカウント
        self.scan_count += 1

        # 指定したスキャン回数に達した場合、データを JSON ファイルに書き込む
        if self.scan_count >= self.max_scans:
            # JSONファイルにデータを書き込む
            with open(self.json_file_path, 'w') as json_file:
                json.dump(self.data_storage, json_file)
            self.get_logger().info(f'Data written to {self.json_file_path}: {self.data_storage}')
            
            # データをリセットして次のバッチのためにカウントを再開
            self.data_storage = []
            self.scan_count = 0

def main(args=None):
    rclpy.init(args=args)
    node = SensorReader(max_scans=10)  # 最大スキャン回数を指定
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import json
import os

class SensorReader(Node):
    def __init__(self):
        super().__init__('sensor_reader')
        
        # 'config' フォルダが存在しない場合は作成する
        if not os.path.exists('config'):
            os.makedirs('config')
        
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # データをため込むリスト
        self.data_storage = []

        # JSONファイルの準備
        self.json_file_path = 'config/sensor_data.json'
        self.json_file = open(self.json_file_path, mode='w')

    def listener_callback(self, msg):
        # センサー値を取得してリストに追加
        ranges = list(msg.ranges)  # arrayをリストに変換
        timestamp = self.get_clock().now().to_msg().sec  # タイムスタンプを取得
        self.data_storage.append({'timestamp': timestamp, 'ranges': ranges})
        json.dump(self.data_storage, self.json_file)
        self.json_file.flush()  # データをすぐにファイルに書き込む
        self.get_logger().info(f'Data stored: {ranges}')

    def __del__(self):
        # ノードが終了するときにファイルを閉じる
        self.json_file.close()

def main(args=None):
    rclpy.init(args=args)
    node = SensorReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""


"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class SensorReader(Node):
    def __init__(self):
        super().__init__('sensor_reader')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # センサー値を取得して表示する
        ranges = msg.ranges
        self.get_logger().info(f'Received scan data: {ranges}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""