import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import json
import time
import os
import math

class Turtlebot3Control(Node):
    def __init__(self):
        super().__init__('turtlebot3_control')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscriber_ = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.imu_subscriber_ = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.declare_parameter('json_file', 'config/commands.json')
        self.timer_callback_executed = False
        self.last_modified_time = 0.0
        self.current_command_index = 0  # 現在のコマンドのインデックス
        self.commands = []  # JSONファイルから読み込んだコマンドを保存

        # 状態変数
        self.current_position = (0.0, 0.0)
        self.current_orientation = 0.0  # IMUから取得する角度
        self.current_pose = None  # Odometryからの位置情報を保存するための変数

    def odom_callback(self, msg):
        # Odometry情報からロボットの位置を取得
        position = msg.pose.pose.position
        self.current_pose = (position.x, position.y)
        self.current_position = (position.x, position.y)

    def imu_callback(self, msg):
        # IMU情報からロボットの向きを取得
        orientation = msg.orientation
        self.current_orientation = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)

    def quaternion_to_euler(self, x, y, z, w):
        # 四元数をオイラー角（ラジアン）に変換
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def move_distance(self, direction, target_distance):
        if self.current_pose is None:
            self.get_logger().error('Current pose is not available.')
            return

        twist = Twist()
        if direction == 'forward':
            twist.linear.x = 0.2
        elif direction == 'backward':
            twist.linear.x = -0.2

        self.publisher_.publish(twist)
        self.get_logger().info(f'Moving {direction} for {target_distance} meters')

        initial_position = self.current_pose
        while self.current_pose is not None:
            current_distance = self.calculate_distance_traveled(initial_position)
            if current_distance >= target_distance:
                break
            time.sleep(0.1)
        
        # ロボットを停止
        twist = Twist()  # Twistメッセージをリセットして停止
        self.publisher_.publish(twist)

    def calculate_distance_traveled(self, initial_position):
        if self.current_pose is None:
            return 0.0
        initial_x, initial_y = initial_position
        current_x, current_y = self.current_pose
        return math.sqrt((current_x - initial_x)**2 + (current_y - initial_y)**2)

    def timer_callback(self):
        json_file = 'config/commands.json'
        
        # ファイルの最終更新時刻を取得
        current_modified_time = os.path.getmtime(json_file)
        
        # ファイルが変更されたかを確認
        if current_modified_time != self.last_modified_time:
            self.last_modified_time = current_modified_time
            self.timer_callback_executed = False
            self.current_command_index = 0  # インデックスをリセット

            # JSONファイルを再読み込み
            with open(json_file, 'r') as file:
                self.commands = json.load(file)['commands']
        
        if self.timer_callback_executed or self.current_command_index >= len(self.commands):
            return

        # 現在のコマンドを取得
        command = self.commands[self.current_command_index]

        if 'distance' in command:
            # 移動コマンド
            target_distance = float(command['distance'].replace('cm', '')) / 100.0
            self.move_distance(command['direction'], target_distance)
        
        elif 'angle' in command:
            # 回転コマンド
            target_angle = float(command['angle'])
            twist = Twist()
            if command['direction'] == 'left':
                twist.angular.z = 0.2
            elif command['direction'] == 'right':
                twist.angular.z = -0.2

            self.get_logger().info(f'Rotating {command["direction"]} for {target_angle} degrees')
            self.publisher_.publish(twist)

            # フィードバック制御
            while True:
                current_angle = math.degrees(self.current_orientation)
                if abs(current_angle - target_angle) < 5.0:  # 目標角度に近づいたら停止
                    break
                time.sleep(0.1)

        # ロボットを停止
        twist = Twist()  # Twistメッセージをリセットして停止
        self.publisher_.publish(twist)
        
        # 次のコマンドに進む前に少し待つ
        time.sleep(1)  # 1秒待つことで、次の動作前にロボットが確実に停止する時間を確保

        # 次のコマンドに進む
        self.current_command_index += 1

        # 全てのコマンドが実行されたらフラグを設定
        if self.current_command_index >= len(self.commands):
            self.timer_callback_executed = True

def main(args=None):
    rclpy.init(args=args)
    turtlebot3_control = Turtlebot3Control()
    rclpy.spin(turtlebot3_control)
    turtlebot3_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
