import rclpy
from rclpy.node import Node
from person_msgs.msg import RobotStatus
import time

class StatusPublisher(Node):
    def __init__(self):
        super().__init__('status_publisher')
        self.publisher = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)
        self.start_time = time.time() #稼働時間の計測開始
        self.mode = 'Idle' #初期モードは待機中

    def publish_status(self):
        msg = RobotStatus()
        msg.battery_level = 75.0 #75%のバッテリー残量
        msg.position = "x: 1.2, y: 3.4, z: 5.6" #ロボットの位置
        msg.speed = 0.5 #ロボットの速度（m/s）
        msg.uptime = time.time() - self.start_time #ロボットの稼働時間
        msg.error_status = "No errors" #エラーステータス
        msg.mode = self.mode #作業モード

        #モードの更新（1分ごとにランダムにモード変更）
        if int(msg.uptime) % 60 == 0:
            self.mode = 'Active' if self.mode == 'Idle' else 'Idle'

        self.publisher.publish(msg)
        self.get_logger().info(f'Published robot status: Battery: {msg.battery_level}, '
                               f'Position: {msg.position}, Speed: {msg.speed}, '
                               f'Uptime: {msg.uptime}, Error: {msg.error_status}, Mode: {msg.mode}')

def main(args=None):
    rclpy.init(args=args)
    node = StatusPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

