# ロボットの状態監視システム

Robot Status Monitoring Systemは、ROS 2を使用してロボットの状態（バッテリー残量、位置、速度、動作モード）を定期的にパブリッシュするシステムです。

---

# 動作環境
このパッケージは以下の環境で動作が確認済み

- OS: Ubuntu 22.04 LTS
- ROS 2 バージョン名: foxy

---

## 機能

## 1.ステータスパブリッシャー
   ノードstatus_publisherがロボットの状態をトピック/robot_statusに定期的にパブリッシュします。
   - バッテリー残量
   - 現在の位置（x, y, z座標）
   - 速度
   - 動作モード（Idle/Active）
   - エラー状態（デフォルトは"No errors"）

## 2.GitHub Actions CI  
   - Pythonコードのスタイルチェック（flake8, pep257）
   - SPDXライセンス表記の検証
   - ROS2のビルドとテスト

---

## インストール方法
以下の手順でこのパッケージをセットアップできます。

## 1.リポジトリのクローン

   ```
   git clone https://github.com/<your-username>/Robot-status-monitoring-system.git
   cd Robot-status-monitoring-system
   ```

## 2.ビルド

   ```
   colcon build --symlink-install
   source install/setup.bash
   ```

---

# 使用方法

## 1.デフォルトでは、status_publisherを実行できます
   ```
ros2 run mypkg status_publisher
   ```
出力
   ```
[INFO] [1736073956.632897448] [status_publisher]: ロボットの状態を公開しました: バッテリー: 75.0%, 位置: x: 1.2, y: 3.4, z: 5.6, 速度: 0.5 m/s, 稼働時間: 1.00 秒, エラー: 問題なし, モード: Idle
[INFO] [1736073957.618620834] [status_publisher]: ロボットの状態を公開しました: バッテリー: 75.0%, 位置: x: 1.2, y: 3.4, z: 5.6, 速度: 0.5 m/s, 稼働時間: 2.00 秒, エラー: 問題なし, モード: Idle
[INFO] [1736073958.618916462] [status_publisher]: ロボットの状態を公開しました: バッテリー: 75.0%, 位置: x: 1.2, y: 3.4, z: 5.6, 速度: 0.5 m/s, 稼働時間: 3.00 秒, エラー: 問題なし, モード: Idle
[INFO] [1736073959.620115984] [status_publisher]: ロボットの状態を公開しました: バッテリー: 75.0%, 位置: x: 1.2, y: 3.4, z: 5.6, 速度: 0.5 m/s, 稼働時間: 4.00 秒, エラー: 問題なし, モード: Idle
[INFO] [1736073960.619073883] [status_publisher]: ロボットの状態を公開しました: バッテリー: 75.0%, 位置: x: 1.2, y: 3.4, z: 5.6, 速度: 0.5 m/s, 稼働時間: 5.00 秒, エラー: 問題なし, モード: Idle
   ```
## 1.Launchファイルを使った起動
   パブリッシャーノードを起動するには、以下のコマンドを実行してください。
   ```
   ros2 launch mypkg status_publisher_launch.py
   ```
出力
   ```
[INFO] [status_publisher-1]: process started with pid [15206]
[status_publisher-1] [INFO] [1736074046.651301010] [status_publisher]: ロボ ットの状態を公開しました: バッテリー: 75.0%, 位置: x: 1.2, y: 3.4, z: 5.6,  速度: 0.5 m/s, 稼働時間: 1.00 秒, エラー: 問題なし, モード: Idle
[status_publisher-1] [INFO] [1736074047.639295454] [status_publisher]: ロボ ットの状態を公開しました: バッテリー: 75.0%, 位置: x: 1.2, y: 3.4, z: 5.6,  速度: 0.5 m/s, 稼働時間: 2.00 秒, エラー: 問題なし, モード: Idle
[status_publisher-1] [INFO] [1736074048.638868550] [status_publisher]: ロボ ットの状態を公開しました: バッテリー: 75.0%, 位置: x: 1.2, y: 3.4, z: 5.6,  速度: 0.5 m/s, 稼働時間: 3.00 秒, エラー: 問題なし, モード: Idle
[status_publisher-1] [INFO] [1736074049.639246648] [status_publisher]: ロボ ットの状態を公開しました: バッテリー: 75.0%, 位置: x: 1.2, y: 3.4, z: 5.6,  速度: 0.5 m/s, 稼働時間: 4.00 秒, エラー: 問題なし, モード: Idle
[status_publisher-1] [INFO] [1736074050.641799707] [status_publisher]: ロボ ットの状態を公開しました: バッテリー: 75.0%, 位置: x: 1.2, y: 3.4, z: 5.6,  速度: 0.5 m/s, 稼働時間: 5.00 秒, エラー: 問題なし, モード: Idle
   ```


## 2.トピックの確認
   パブリッシュされたデータを確認するには、以下を実行してください。
   ```
   ros2 topic echo /robot_status
   ```
出力
   ```
data: 'バッテリー: 75.0%, 位置: x: 1.2, y: 3.4, z: 5.6, 速度: 0.5 m/s, 稼働 時間: 5.00 秒, エラー: 問題なし, モード: Idle'
---
data: 'バッテリー: 75.0%, 位置: x: 1.2, y: 3.4, z: 5.6, 速度: 0.5 m/s, 稼働 時間: 6.00 秒, エラー: 問題なし, モード: Idle'
---
data: 'バッテリー: 75.0%, 位置: x: 1.2, y: 3.4, z: 5.6, 速度: 0.5 m/s, 稼働 時間: 7.00 秒, エラー: 問題なし, モード: Idle'
---
data: 'バッテリー: 75.0%, 位置: x: 1.2, y: 3.4, z: 5.6, 速度: 0.5 m/s, 稼働 時間: 8.00 秒, エラー: 問題なし, モード: Idle'
---
data: 'バッテリー: 75.0%, 位置: x: 1.2, y: 3.4, z: 5.6, 速度: 0.5 m/s, 稼働 時間: 9.00 秒, エラー: 問題なし, モード: Idle'
---
data: 'バッテリー: 75.0%, 位置: x: 1.2, y: 3.4, z: 5.6, 速度: 0.5 m/s, 稼働 時間: 10.00 秒, エラー: 問題なし, モード: Idle'
---
data: 'バッテリー: 75.0%, 位置: x: 1.2, y: 3.4, z: 5.6, 速度: 0.5 m/s, 稼働 時間: 11.00 秒, エラー: 問題なし, モード: Idle'
   ```

---

# ライセンス
-  このソフトウェアパッケージは、3条項BSDライセンスの下、再頒布および使用が許可されます。
- © 2024 Suzuki Takuma
