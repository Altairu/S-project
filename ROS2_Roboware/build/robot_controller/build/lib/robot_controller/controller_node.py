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

        # キーボード操作の最大値
        self.max_x = 80
        self.max_y = 40
        self.max_z = 15

        # 操作モードフラグ (True: GUI, False: キーボード)
        self.gui_mode = True

        # 座標マッピング（各ボタンに対応するX, Y座標）
        self.positions = {
            '1': (0, 0),
            '2': (50, 0),
            '3': (100, 0),
            '4': (0, 50),
            '5': (50, 50),
            '6': (100, 50)
        }

        # GUIの設定
        self.root = tk.Tk()
        self.root.title("Position Controller")
        self.root.geometry("800x400")
        
        # ウィンドウを常に前面に表示
        self.root.attributes('-topmost', True)

        # 操作モードトグルスイッチ
        self.gui_mode_var = tk.BooleanVar(value=True)
        toggle_button = tk.Checkbutton(self.root, text="GUI Mode", variable=self.gui_mode_var, command=self.toggle_mode)
        toggle_button.grid(row=0, column=3, padx=20, pady=10)

        # 座標設定ボタンの配置 (GUI モード用)
        for i, (pos, (x, y)) in enumerate(self.positions.items(), start=1):
            button = tk.Button(self.root, text=str(i), font=("Arial", 20), width=4, height=2,
                               command=lambda x=x, y=y: self.update_position(x=x, y=y))
            button.grid(row=(i-1)//3, column=(i-1)%3, padx=20, pady=20)

        # Z方向のボタン
        z_label = tk.Label(self.root, text="Z Position", font=("Arial", 16))
        z_label.grid(row=0, column=4, padx=20, pady=10)

        tk.Button(self.root, text="Z = 0", font=("Arial", 15), width=6,
                  command=lambda: self.update_position(z=0)).grid(row=1, column=4, padx=10, pady=10)
        tk.Button(self.root, text="Z = 50", font=("Arial", 15), width=6,
                  command=lambda: self.update_position(z=50)).grid(row=2, column=4, padx=10, pady=10)

        # 現在の位置表示ラベル
        self.position_label = tk.Label(self.root, text=f"Current Position: X={self.x}, Y={self.y}, Z={self.z}", font=("Arial", 15))
        self.position_label.grid(row=3, column=0, columnspan=5, pady=20)

        # キーボード操作用のキーイベントをバインド
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)

        # 長押し時のキー判定用フラグ
        self.keys_pressed = set()

        # GUIループ開始
        self.root.after(100, self.update_position_continuous)
        self.root.mainloop()

    def toggle_mode(self):
        # GUI モードとキーボードモードの切り替え
        self.gui_mode = self.gui_mode_var.get()
        self.keys_pressed.clear()  # キーボードモードに入る前に押されたキーをクリア

    def update_position(self, x=None, y=None, z=None):
        # 座標の更新 (GUI もしくはキーボード操作で呼ばれる)
        if x is not None:
            self.x = max(0, min(self.max_x, x))
        if y is not None:
            self.y = max(0, min(self.max_y, y))
        if z is not None:
            self.z = max(0, min(self.max_z, z))

        # 現在の位置を表示
        self.position_label.config(text=f"Current Position: X={self.x}, Y={self.y}, Z={self.z}")

        # トピックに現在の位置を送信
        msg = Int16MultiArray()
        msg.data = [self.x, self.y, self.z]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published position: X={self.x}, Y={self.y}, Z={self.z}")

    def on_key_press(self, event):
        if not self.gui_mode:
            self.keys_pressed.add(event.keysym)

    def on_key_release(self, event):
        if not self.gui_mode:
            self.keys_pressed.discard(event.keysym)

    def update_position_continuous(self):
        # キーボード操作による長押し対応
        if not self.gui_mode:
            if 'a' in self.keys_pressed and self.x > 0:
                self.update_position(x=self.x - 1)
            if 'd' in self.keys_pressed and self.x < self.max_x:
                self.update_position(x=self.x + 1)
            if 'w' in self.keys_pressed and self.y < self.max_y:
                self.update_position(y=self.y + 1)
            if 's' in self.keys_pressed and self.y > 0:
                self.update_position(y=self.y - 1)
            if 'r' in self.keys_pressed and self.z < self.max_z:
                self.update_position(z=self.z + 1)
            if 'f' in self.keys_pressed and self.z > 0:
                self.update_position(z=self.z - 1)
        
        # 100ms毎に状態を更新
        self.root.after(50, self.update_position_continuous)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
