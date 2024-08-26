import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json
import time
import os

class Turtlebot3Control(Node):
    def __init__(self):
        super().__init__('turtlebot3_control')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.declare_parameter('json_file', 'config/commands.json')
        self.timer_callback_executed = False
        self.last_modified_time = 0.0
        self.current_command_index = 0  # 現在のコマンドのインデックス
        self.commands = []  # JSONファイルから読み込んだコマンドを保存

        # 設定ファイルのパスを構築
        config_dir = os.path.expanduser('~/turtlebot3_ws/src/turtlebot3_control/config')
        self.json_file_path = os.path.join(config_dir, "commands.json")

    def timer_callback(self):
        json_file = 'config/commands.json'
        
        # ファイルの最終更新時刻を取得
        current_modified_time = os.path.getmtime(json_file_path)
        
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
        twist = Twist()

        if 'distance' in command:
            # 移動コマンド
            if command['direction'] == 'forward':
                twist.linear.x = 0.5
            elif command['direction'] == 'backward':
                twist.linear.x = -0.5
            
            distance = float(command['distance'].replace('cm', '')) / 100.0
            self.get_logger().info(f'Moving {command["direction"]} for {distance} meters')
            self.publisher_.publish(twist)
            time.sleep(distance / 0.1)  # 距離に応じてスリープする（速度0.1 m/sを仮定）

        elif 'angle' in command:
            # 回転コマンド
            angle = float(command['angle'])
            if command['direction'] == 'left':
                twist.angular.z = 0.5
            elif command['direction'] == 'right':
                twist.angular.z = -0.5
            
            rotation_time = angle / 45.0  # 45度回転するのに1秒かかると仮定
            self.get_logger().info(f'Rotating {command["direction"]} for {angle} degrees')
            self.publisher_.publish(twist)
            time.sleep(rotation_time)  # 角度に応じてスリープする

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
