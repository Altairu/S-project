import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

class RvizPositionNode(Node):
    def __init__(self):
        super().__init__('rviz_position_node')

        # トピックのサブスクライバ設定
        self.subscription = self.create_subscription(
            Int16MultiArray,
            'current_position',
            self.position_callback,
            10
        )

    def position_callback(self, msg):
        x_mm, y_mm, z_mm = msg.data
        self.get_logger().info(f"Position in RViz: X={x_mm}, Y={y_mm}, Z={z_mm}")

def main(args=None):
    rclpy.init(args=args)
    node = RvizPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
