import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import serial
import struct

class SerialReadNode(Node):
    def __init__(self):
        super().__init__('serial_read_node')

        # シリアルポート設定
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        
        # トピックのパブリッシャ設定
        self.publisher_ = self.create_publisher(Int16MultiArray, 'current_position', 10)

        # タイマー設定
        self.timer = self.create_timer(0.5, self.read_position_callback)

    def read_position_callback(self):
        if self.serial_port.in_waiting >= 10:
            data = self.serial_port.read(10)

            if data[:2] == b'\xA5\xA5' and data[-2:] == b'\x5A\x5A':
                x_mm, y_mm, z_mm = struct.unpack('>hhh', data[2:8])
                msg = Int16MultiArray()
                msg.data = [x_mm, y_mm, z_mm]
                self.publisher_.publish(msg)
                self.get_logger().info(f"Received position: X={x_mm}, Y={y_mm}, Z={z_mm}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialReadNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
