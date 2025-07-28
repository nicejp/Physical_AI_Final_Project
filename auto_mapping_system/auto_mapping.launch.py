#!/usr/bin/env python3

# ===== TurtleBot3 自動マッピングシステム 統合起動ファイル =====
# Physical AI 最終課題用のlaunchファイル
#
# このファイルの役割：
# 複数のROS2ノードを同時に起動して、完全な自動マッピングシステムを構築
# 
# 起動されるもの：
# 1. Gazebo（物理シミュレーション環境）
# 2. TurtleBot3 House（家の中の環境）
# 3. Cartographer（SLAMアルゴリズム）
# 4. RViz（可視化ツール）
# 5. 自動探索ノード（私たちが作ったAI）
#
# 学習ポイント：
# - ROS2 Launchシステムの使い方
# - 複数ノードの協調動作
# - タイミング制御（順番に起動）
# =================================================================

# ROS2 Launchシステムの必要なモジュールをインポート
from launch import LaunchDescription                                                     # Launchファイルの基本クラス
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction  # Launch時のアクション
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution               # 設定値とパス処理
from launch_ros.actions import Node                                                      # ROS2ノード起動用
from launch_ros.substitutions import FindPackageShare                                    # パッケージ検索用
from launch.launch_description_sources import PythonLaunchDescriptionSource              # Pythonベースのラウンチファイル読み込み

def generate_launch_description():
    """
    Launch設定を生成する関数
    
    この関数がlaunchファイルの本体。
    どのノードをどの順番で起動するかを定義している。
    
    ROS2では、この関数名は固定で、必ずこの名前にする必要がある。
    Launch時に自動的にこの関数が呼び出される。
    """
    
    # ===== Launch設定変数 =====
    # シミュレーション時間を使用するかどうかの設定
    # 'true'にすることで、Gazeboの時間とROS2の時間が同期される
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # ===== 1. Gazebo + TurtleBot3 House環境の起動 =====
    # Gazebo：物理シミュレーションエンジン（重力、衝突、摩擦などを再現）
    # TurtleBot3 House：家の中のような環境（壁、家具、部屋がある）
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),      # turtlebot3_gazeboパッケージを探す
                'launch',                                   # launchディレクトリ
                'turtlebot3_house.launch.py'                # house環境のlaunchファイル
            ])
        ])
    )
    
    # ===== 2. Cartographer SLAM の起動 =====
    # Cartographer：Googleが開発したSLAMアルゴリズム
    # SLAM = Simultaneous Localization and Mapping
    # → 同時に自己位置推定と地図作成を行う技術
    #
    # 動作原理：
    # - LiDARセンサーで周囲をスキャン
    # - 移動しながら各地点での観測データを蓄積
    # - データを統合して一貫性のある地図を作成
    # - 同時に、作成中の地図上での自分の位置も推定
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_cartographer'),     # cartographerパッケージを探す
                'launch',
                'cartographer.launch.py'                         # cartographerのlaunchファイル
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()  # シミュレーション時間設定を渡す
    )
    
    # ===== 3. RViz 可視化ツールの起動 =====
    # RViz：ROS2の標準的な3D可視化ツール
    # 以下のものがリアルタイムで見られる：
    # - ロボットの現在位置と姿勢
    # - LiDARセンサーのスキャンデータ（赤い点群）
    # - 作成されていく地図（グレーの格子）
    # - ロボットの移動軌跡
    
    # Cartographer用の設定ファイルを指定
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('turtlebot3_cartographer'),
        'rviz',                                     # rvizディレクトリ
        'tb3_cartographer.rviz'                     # Cartographer用の設定ファイル
    ])
    
    rviz_node = Node(
        package='rviz2',                            # RViz2パッケージ
        executable='rviz2',                         # rviz2実行ファイル
        name='rviz2',                               # ノード名
        arguments=['-d', rviz_config_file],         # 設定ファイルを指定
        parameters=[{'use_sim_time': use_sim_time}],# シミュレーション時間設定
        output='screen'                             # 出力をターミナルに表示
    )
    
    # ===== 4. 自動探索ノードの起動（遅延起動） =====
    # 私たちが作成したAI探索ノード
    # 他のシステム（Gazebo、SLAM、RViz）が完全に立ち上がってから
    # 5秒後に起動するように設定
    #
    # 遅延起動する理由：
    # - Gazeboの世界が完全に読み込まれるまで待つ
    # - TurtleBot3のセンサーが準備完了するまで待つ
    # - SLAMシステムが初期化されるまで待つ
    # → 全てが準備完了してから探索を開始することで安定動作を実現
    auto_explorer_node = TimerAction(
        period=5.0,  # 5秒待つ
        actions=[
            Node(
                package='auto_mapping_system',          # 私たちのパッケージ名
                executable='auto_explorer',             # 実行ファイル名（setup.pyで定義）
                name='auto_mapping_explorer',           # ノード名
                parameters=[{'use_sim_time': use_sim_time}], # シミュレーション時間設定
                output='screen'                         # ログをターミナルに表示
            )
        ]
    )
    
    # ===== Launch設定の組み立て =====
    # LaunchDescriptionオブジェクトを作成し、すべての起動要素を登録
    # 
    # 起動順序：
    # 1. DeclareLaunchArgument: コマンドライン引数の定義
    # 2. gazebo_launch: Gazebo + TurtleBot3 House（即座に起動）
    # 3. cartographer_launch: SLAM システム（即座に起動）
    # 4. rviz_node: 可視化ツール（即座に起動）
    # 5. auto_explorer_node: AI探索ノード（5秒後に起動）
    return LaunchDescription([
        # ===== コマンドライン引数の定義 =====
        # launch時に --use_sim_time=false のように指定可能
        # 通常はデフォルト値'true'が使用される
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time for all nodes'  # この引数の説明
        ),
        
        # ===== 各ノード・システムの起動登録 =====
        # ここに登録された順番で起動される
        gazebo_launch,          # Gazebo物理シミュレーション環境
        cartographer_launch,    # SLAMアルゴリズム
        rviz_node,              # 3D可視化ツール
        auto_explorer_node      # AI探索ノード（5秒遅延）
    ])
