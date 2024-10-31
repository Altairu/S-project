import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import tkinter as tk

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # ROS2のパブリッシャ
        self.publisher_ = self.create_publisher(Int16MultiArray, 'target_position', 10)

        # 初期位置の設定
        self.x = 0
        self.y = 0
        self.z = 0

        # 座標マッピング（各ボタンに対応するX, Y座標）
        self.positions = {
            '1': (0, 40),
            '2': (40, 40),
            '3': (80, 40),
            '4': (0, 0),
            '5': (40, 0),
            '6': (80, 0)
        }

        # GUIの設定
        self.root = tk.Tk()
        self.root.title("Position Controller")
        self.root.geometry("600x400")
        
        # ウィンドウを常に前面に表示
        self.root.attributes('-topmost', True)

        # 座標設定ボタンの配置
        for i, (pos, (x, y)) in enumerate(self.positions.items(), start=1):
            button = tk.Button(self.root, text=str(i), font=("Arial", 20), width=4, height=2,
                               command=lambda x=x, y=y: self.update_position(x, y))
            button.grid(row=(i-1)//3, column=(i-1)%3, padx=20, pady=20)

        # Z方向のボタン
        z_label = tk.Label(self.root, text="Z Position", font=("Arial", 16))
        z_label.grid(row=0, column=3, padx=20, pady=10)

        tk.Button(self.root, text="Z = 10", font=("Arial", 15), width=6,
                  command=lambda: self.update_position(z=10)).grid(row=1, column=3, padx=10, pady=10)
        tk.Button(self.root, text="Z = 0", font=("Arial", 15), width=6,
                  command=lambda: self.update_position(z=0)).grid(row=2, column=3, padx=10, pady=10)

        # 現在の位置表示ラベル
        self.position_label = tk.Label(self.root, text=f"Current Position: X={self.x}, Y={self.y}, Z={self.z}", font=("Arial", 15))
        self.position_label.grid(row=3, column=0, columnspan=4, pady=20)

        # GUIループ開始
        self.root.mainloop()

    def update_position(self, x=None, y=None, z=None):
        # 座標の更新
        if x is not None:
            self.x = x
        if y is not None:
            self.y = y
        if z is not None:
            self.z = z

        # 現在の位置を表示
        self.position_label.config(text=f"Current Position: X={self.x}, Y={self.y}, Z={self.z}")

        # トピックに現在の位置を送信
        msg = Int16MultiArray()
        msg.data = [self.x, self.y, self.z]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published position: X={self.x}, Y={self.y}, Z={self.z}")

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
