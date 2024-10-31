import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Int16MultiArray

class RVizPositionNode(Node):
    def __init__(self):
        super().__init__('rviz_position_node')

        # サブスクライバ：現在位置を受信
        self.current_position_sub = self.create_subscription(
            Int16MultiArray,
            'current_position',
            self.current_position_callback,
            10
        )

        # サブスクライバ：目標位置を受信
        self.target_position_sub = self.create_subscription(
            Int16MultiArray,
            'target_position',
            self.target_position_callback,
            10
        )

        # パブリッシャ：現在位置の可視化用マーカー
        self.current_marker_pub = self.create_publisher(Marker, 'current_position_marker', 10)
        
        # パブリッシャ：目標位置の可視化用マーカー
        self.target_marker_pub = self.create_publisher(Marker, 'target_position_marker', 10)

        # 初期化
        self.current_x = 0
        self.current_y = 0
        self.current_z = 0
        self.target_x = 0
        self.target_y = 0
        self.target_z = 0

    def current_position_callback(self, msg):
        # 現在位置を更新
        self.current_x, self.current_y, self.current_z = msg.data
        self.publish_current_position_marker()

    def target_position_callback(self, msg):
        # 目標位置を更新
        self.target_x, self.target_y, self.target_z = msg.data
        self.publish_target_position_marker()

    def publish_current_position_marker(self):
        # 現在位置のマーカーを作成してパブリッシュ
        current_marker = Marker()
        current_marker.header.frame_id = "map"
        current_marker.header.stamp = self.get_clock().now().to_msg()
        current_marker.type = Marker.SPHERE
        current_marker.action = Marker.ADD
        current_marker.pose.position.x = float(self.current_x)
        current_marker.pose.position.y = float(self.current_y)
        current_marker.pose.position.z = float(self.current_z)
        current_marker.scale.x = 0.05  # 小さなスケールに調整
        current_marker.scale.y = 0.05
        current_marker.scale.z = 0.05
        current_marker.color.a = 1.0
        current_marker.color.r = 0.0
        current_marker.color.g = 1.0
        current_marker.color.b = 0.0
        self.current_marker_pub.publish(current_marker)

    def publish_target_position_marker(self):
        # 目標位置のマーカーを作成してパブリッシュ
        target_marker = Marker()
        target_marker.header.frame_id = "map"
        target_marker.header.stamp = self.get_clock().now().to_msg()
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        target_marker.pose.position.x = float(self.target_x)
        target_marker.pose.position.y = float(self.target_y)
        target_marker.pose.position.z = float(self.target_z)
        target_marker.scale.x = 0.05  # 小さなスケールに調整
        target_marker.scale.y = 0.05
        target_marker.scale.z = 0.05
        target_marker.color.a = 1.0
        target_marker.color.r = 1.0
        target_marker.color.g = 0.0
        target_marker.color.b = 0.0
        self.target_marker_pub.publish(target_marker)


def main(args=None):
    rclpy.init(args=args)
    node = RVizPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
