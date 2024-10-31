# robot_controller パッケージ

## 概要

`robot_controller` パッケージは、PC 上の GUI を通じてロボットの目標位置を設定し、シリアル通信や CAN 通信を介してロボットの動作を制御するための ROS2 パッケージです。このパッケージには、ユーザーが座標を選択できる操縦用 GUI、シリアル通信を用いた目標位置の送信、受信位置の可視化などの機能が含まれています。

## システム構成

このパッケージには以下の 4 つのノードが含まれています。

1. **controller_node**: GUI インターフェースを提供し、ユーザーが目標座標（X, Y, Z）を設定できる。
2. **serial_send_node**: `controller_node` から送られてきた目標位置をシリアル通信を介してロボットに送信する。
3. **serial_read_node**: シリアル通信でロボットから現在位置データを受信し、トピックに配信する。
4. **rviz_position_node**: `serial_read_node` から受信した現在位置データを ROS2 トピックを介して受け取り、RViz またはログで現在位置を確認する。

---

## 各ノードの説明

### 1. controller_node

- GUI インターフェースを通じてロボットの目標座標を設定します。
- `target_position` トピックに目標位置（X, Y, Z）を配信し、`serial_send_node` に目標位置を渡します。

### 2. serial_send_node

- `target_position` トピックから目標位置を受信し、シリアル通信でロボットに送信します。
- 定期的に目標位置を送信し、常に最新の目標位置がロボットに伝わるようにします。

### 3. serial_read_node

- シリアル通信でロボットから現在の位置データを受信します。
- `current_position` トピックに現在位置（X, Y, Z）を配信し、他のノードが利用できるようにします。

### 4. rviz_position_node

- `current_position` トピックからロボットの現在位置を受信します。
- 受信した現在位置をログに表示し、必要に応じて RViz 上で確認できるようにします。

---

## セットアップ

### 依存パッケージのインストール

1. `ROS2`（`rclpy`, `std_msgs` パッケージが含まれていることを確認してください）。
2. `pyserial` パッケージは、シリアル通信に必要です。以下のコマンドでインストールできます。

   ```bash
   pip install pyserial
   ```

### パッケージのビルド

ROS2 ワークスペース内に `robot_controller` パッケージを配置し、以下のコマンドでビルドします。

```bash
cd ~/ros2_ws
colcon build --packages-select robot_controller
```

### 環境変数の設定

パッケージを利用するために、ワークスペースのセットアップスクリプトを読み込みます。

```bash
source ~/ros2_ws/install/setup.bash
```

---

## ノードの起動方法

### 1. controller_node の起動

`controller_node` を起動すると、GUI ウィンドウが開き、ロボットの目標座標を設定できます。

```bash
ros2 run robot_controller controller_node
```

### 2. serial_send_node の起動

シリアル通信で目標座標をロボットに送信するために `serial_send_node` を起動します。

```bash
ros2 run robot_controller serial_send_node
```

### 3. serial_read_node の起動

`serial_read_node` はロボットから現在位置を受信し、トピックに配信します。

```bash
ros2 run robot_controller serial_read_node
```

### 4. rviz_position_node の起動

現在位置を `RViz` で可視化するために `rviz_position_node` を起動します。

```bash
ros2 run robot_controller rviz_position_node
```

---

## トピック

- **`target_position` (std_msgs/Int16MultiArray)**: `controller_node` が発行するトピック。目標位置（X, Y, Z）を送信します。
- **`current_position` (std_msgs/Int16MultiArray)**: `serial_read_node` が発行するトピック。ロボットの現在位置（X, Y, Z）を配信します。

---

## 動作確認

1. `controller_node` で GUI を開き、目標位置を設定してください。設定した位置が `target_position` トピックに配信されます。
2. `serial_send_node` を起動すると、`target_position` トピックから目標位置を取得し、シリアル通信でロボットに送信します。
3. ロボットが位置を変更したら、`serial_read_node` が現在位置を受信して `current_position` トピックに配信します。
4. `rviz_position_node` を使用して `current_position` トピックのデータを確認できます。
