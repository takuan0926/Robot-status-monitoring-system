#!/usr/bin/python3
# SPDX-FileCopyrightText: 2024 suzuki takuma <s23c1076vc@s.chibakoudai.jp>
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import String

class StatusPublisher(Node):
    def __init__(self):
        super().__init__('status_publisher')
        self.publisher = self.create_publisher(String, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)
        self.start_time = time.time() #稼働時間の計測開始
        self.mode = 'Idle' #初期モードは待機中

    def update_status(self):
        #ロボットの状態を更新
        if int(time.time() - self.start_time) % 60 == 0:
            self.mode = 'Active' if self.mode == 'Idle' else 'Idle'

    def create_message(self):
        #ロボットの状態メッセージを作成
        msg = String()
        msg.data = f"バッテリー: 75.0%, 位置: x: 1.2, y: 3.4, z: 5.6, " \
                   f"速度: 0.5 m/s, 稼働時間: {time.time() - self.start_time:.2f} 秒, " \
                   f"エラー: 問題なし, モード: {self.mode}"
        return msg

    def publish_status(self):
        #状態を更新し、メッセージをパブリッシュ
        self.update_status()
        msg = self.create_message()  
        self.publisher.publish(msg)
        self.get_logger().info(f"ロボットの状態を公開しました: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = StatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

