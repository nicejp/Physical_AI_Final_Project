# TurtleBot3 Auto Mapping System - Physical AI Final Project

## 概要
TurtleBot3を使用したAI駆動の自律探索アルゴリズムによる自動マッピングシステムです。完全自動でロボットが環境を探索し、リアルタイムで地図を作成します。

## ファイル構成
```
auto_mapping_system/
├── README.md                           # このファイル
├── run_final_assignment_container.ps1  # Docker起動スクリプト
├── auto_explorer.py                    # AI探索ノード
├── auto_mapping.launch.py              # 統合起動ファイル
├── package.xml                         # パッケージ設定
└── setup.py                            # セットアップファイル
```

## システム要件

### Windows環境
- Windows 10/11 (WSL2対応)
- Docker Desktop
- PowerShell
- メモリ: 8GB以上推奨 (WSL2設定で調整可能)

### 使用技術
- **ROS2 Humble**
- **Gazebo シミュレーション**
- **Cartographer SLAM**
- **AI駆動探索アルゴリズム**
- **LiDAR センサーフュージョン**

## クイックスタート

### 前提条件の確認
- Docker Desktopがインストール済み
- WSL2が有効化済み

### 1. Docker環境の準備

#### Docker Imageの取得
```powershell
docker pull airobotbook/ros2-desktop-ai-robot-book-humble
```

#### WSL2メモリ設定 (環境に合わせて設定)
`C:\Users\<ユーザー名>\.wslconfig` ファイルを作成/編集:
```ini
[wsl2]
memory=64GB
swap=128GB
```

#### コンテナ起動
PowerShellスクリプト `run_ai_robot_container.ps1` を実行:
```powershell
# スクリプトを実行してコンテナを起動
.\run_ai_robot_container.ps1
```

#### VNC接続
ブラウザで以下にアクセス:
```
http://localhost:6080/
```

### 2. Ubuntu環境の初期設定

#### システムアップデート
```bash
sudo apt-get update
```

#### SSH設定 (必要に応じて)
```bash
sudo apt install openssh-server
sudo vi /etc/ssh/ssh_config
# PermitRootLogin no
# PermitEmptyPasswords no
sudo service ssh start
```

#### ROS2キーの修正 (エラーが出る場合、メッセージのキーを確認！)
```bash
sudo apt-key del F42ED6FBAB17C654
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### 3. 必要パッケージのインストール

#### TurtleBot3関連パッケージ
```bash
sudo apt update
sudo apt install -y ros-humble-turtlebot3*
sudo apt install -y ros-humble-gazebo-*
sudo apt install -y ros-humble-cartographer*
sudo apt install -y ros-humble-nav2*
```

#### 依存関係
```bash
sudo apt install -y python3-colcon-common-extensions
sudo apt install -y python3-numpy python3-opencv
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
```

#### Pythonライブラリ
```bash
pip3 install numpy opencv-python
```

### 4. 環境変数設定

```bash
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> ~/.bashrc
source ~/.bashrc
```

### 5. ワークスペースの作成

```bash
# ワークスペース作成
mkdir -p ~/auto_mapping_ws/src
cd ~/auto_mapping_ws/src

# TurtleBot3シミュレーションパッケージのクローン
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

### 6. 自動マッピングパッケージの作成

```bash
cd ~/auto_mapping_ws/src

# ROS2パッケージ作成
ros2 pkg create --build-type ament_python auto_mapping_system --dependencies rclpy geometry_msgs nav_msgs sensor_msgs
```

### 7. ソースファイルの作成

#### 自動探索ノード
```bash
mkdir -p ~/auto_mapping_ws/src/auto_mapping_system/auto_mapping_system

# auto_explorer.pyを作成 (提供されたコードを使用)
vi ~/auto_mapping_ws/src/auto_mapping_system/auto_mapping_system/auto_explorer.py
chmod +x ~/auto_mapping_ws/src/auto_mapping_system/auto_mapping_system/auto_explorer.py

# __init__.pyを作成
touch ~/auto_mapping_ws/src/auto_mapping_system/auto_mapping_system/__init__.py
```

#### launchファイル
```bash
mkdir -p ~/auto_mapping_ws/src/auto_mapping_system/launch

# auto_mapping.launch.pyを作成 (提供されたコードを使用)
vi ~/auto_mapping_ws/src/auto_mapping_system/launch/auto_mapping.launch.py
chmod +x ~/auto_mapping_ws/src/auto_mapping_system/launch/auto_mapping.launch.py
```

### 8. パッケージ設定ファイル

#### package.xml
```bash
vi ~/auto_mapping_ws/src/auto_mapping_system/package.xml
```

#### setup.py
```bash
vi ~/auto_mapping_ws/src/auto_mapping_system/setup.py
```

#### resourceファイル
```bash
mkdir -p ~/auto_mapping_ws/src/auto_mapping_system/resource
echo "auto_mapping_system" > ~/auto_mapping_ws/src/auto_mapping_system/resource/auto_mapping_system
```

### 9. ビルドと実行

#### ビルド
```bash
cd ~/auto_mapping_ws

# 依存関係の解決
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# ビルド
colcon build --symlink-install

# 環境変数の設定
source install/setup.bash
```

#### 動作確認
```bash
# パッケージが正しく認識されているか確認
ros2 pkg executables auto_mapping_system
# 期待される出力: auto_mapping_system auto_explorer

# パッケージリストで確認
ros2 pkg list | grep auto_mapping
```

## 実行方法

### 起動
```bash
ros2 launch auto_mapping_system auto_mapping.launch.py
```

## 提供ファイルの詳細

### 🔧 run_final_assignment_container.ps1
Dockerコンテナ起動用PowerShellスクリプト
- 解像度: 1920x1080
- 共有メモリ: 64GB
- 必要ポート: 6080 (VNC), 5900 (VNC), 3389 (RDP)

### auto_explorer.py
AI駆動の自律探索ノード
- **センサーフュージョン**: 360度LiDARデータの実時間解析
- **状態機械**: exploring/turning/backing の適応的制御
- **壁追従アルゴリズム**: 効率的な探索戦略
- **スタック検出**: 袋小路からの自動回復

### auto_mapping.launch.py
統合起動ファイル
- Gazebo + TurtleBot3 House環境
- Cartographer SLAM
- RViz可視化
- 5秒遅延での自動探索開始

### package.xml & setup.py
ROS2パッケージ設定ファイル
- 依存関係の定義
- 実行ファイルのエントリーポイント設定

## AI機能と技術的特徴

### AI要素
- **センサーフュージョン**: LiDARデータの実時間解析
- **インテリジェント探索**: 状態機械による適応的行動制御
- **障害物回避**: 動的環境に対応する高度なナビゲーション
- **スタック検出**: 袋小路や困難な状況からの自動回復
- **壁追従**: 効率的な探索のための壁追従アルゴリズム

### 使用センサー
- **LiDAR**: 360度レーザースキャナー（距離測定）
- **オドメトリ**: エンコーダー + IMU（位置・姿勢推定）
- **注意**: カメラは使用していません

## ユーティリティ

### 動画録画
```bash
# 画面録画の準備
sudo apt install -y recordmydesktop

# 録画開始
recordmydesktop --fps=15 --delay=5 --on-the-fly-encoding

# 録画停止: Ctrl+C
```

### マップ保存
```bash
# 探索完了後、マップを保存
ros2 run nav2_map_server map_saver_cli -f ~/generated_house_map

# 保存されるファイル:
# - generated_house_map.pgm (画像ファイル)
# - generated_house_map.yaml (メタデータ)
```

### システム停止
```bash
# すべてのプロセスを停止
Ctrl+C
```

## トラブルシューティング

### 環境変数の問題
```bash
cd ~/auto_mapping_ws
source install/setup.bash
ros2 pkg list | grep auto_mapping
```

### パッケージが認識されない場合
```bash
# クリーンビルド
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

### Docker メモリ不足
WSL2設定ファイル `C:\Users\<ユーザー名>\.wslconfig` でメモリを増量

## 作者
Physical AI講座 最終課題

---

**注意**: このシステムはDocker + WSL2環境で動作確認しました。
