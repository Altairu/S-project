import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import serial
import struct

class SerialSendNode(Node):
    def __init__(self):
        super().__init__('serial_send_node')

        # シリアルポート設定
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        # 位置データの初期化
        self.x_mm = 0
        self.y_mm = 0
        self.z_mm = 0

        # サブスクライバ設定
        self.subscription = self.create_subscription(
            Int16MultiArray,
            'target_position',
            self.update_position_callback,
            10
        )

        # 定期送信用のタイマー設定
        self.timer = self.create_timer(0.1, self.send_position)

    def update_position_callback(self, msg):
        # 位置データの更新
        if len(msg.data) >= 3:
            self.x_mm, self.y_mm, self.z_mm = msg.data[0], msg.data[1], msg.data[2]
            self.get_logger().info(f"Updated position to: X={self.x_mm}, Y={self.y_mm}, Z={self.z_mm}")

    def send_position(self):
        # ヘッダー、データ、フッターを作成
        HEADER = b'\xA5\xA5'
        FOOTER = b'\x5A\x5A'
        data = struct.pack('>hhh', self.x_mm, self.y_mm, self.z_mm)
        packet = HEADER + data + FOOTER

        # シリアルポートから送信
        self.serial_port.write(packet)
        self.get_logger().info(f"Sent position: X={self.x_mm}, Y={self.y_mm}, Z={self.z_mm}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialSendNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
