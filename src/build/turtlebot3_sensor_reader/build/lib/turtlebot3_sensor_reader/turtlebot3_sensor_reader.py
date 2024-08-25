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