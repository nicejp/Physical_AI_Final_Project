#!/usr/bin/env python3

# ===== TurtleBot3 自動マッピングシステム 統合起動ファイル =====
# Physical AI 最終課題用
# 複数のROS2ノードを同時に起動して完全な自動マッピングシステムを構築
# 起動するもの：Gazebo + SLAM + RViz + 自動探索AI
# ===============================================================
# 実行時の動作
# 0秒：Gazebo起動 → TurtleBot3がHouse環境に配置
# 0秒：Cartographer起動 → SLAM準備開始
# 0秒：RViz起動 → 空の地図画面表示
# 5秒：AI探索開始 → ロボット自動探索スタート
# 探索中：リアルタイムで地図が作成される様子をRVizで確認可能
# 完了：家全体の詳細な地図が完成！

# ROS2 Launch システムに必要なライブラリをインポート
from launch import LaunchDescription                                         # Launchファイルの基本クラス
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction  # 起動アクション類
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution   # 設定値とパス処理
from launch_ros.actions import Node                                          # ROS2ノード起動用
from launch_ros.substitutions import FindPackageShare                        # ROS2パッケージ検索用
from launch.launch_description_sources import PythonLaunchDescriptionSource  # Pythonlaunchファイル読み込み用

def generate_launch_description():
    # Launch configuration variables
    # シミュレーション時間設定（Gazeboの時間とROS2の時間を同期）
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 1. Gazebo with TurtleBot3 House environment
    # Gazebo物理シミュレーション + TurtleBot3 House環境を起動
    # House環境：家の中のような3D空間（部屋、壁、家具あり）
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),      # turtlebot3_gazeboパッケージを検索
                'launch',                                   # launchディレクトリ
                'turtlebot3_house.launch.py'                # house環境用launchファイル
            ])
        ])
    )
    
    # 2. Cartographer SLAM
    # Google製SLAMアルゴリズムを起動
    # SLAM = Simultaneous Localization and Mapping（同時位置推定・地図作成）
    # LiDARデータから地図を作成しながら自分の位置も推定
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_cartographer'), # cartographerパッケージを検索
                'launch',
                'cartographer.launch.py'                    # cartographer起動用ファイル
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()  # シミュレーション時間設定を渡す
    )
    
    # 3. RViz for real-time visualization
    # RViz3D可視化ツールの設定
    # 地図作成過程をリアルタイムで視覚的に確認するため
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('turtlebot3_cartographer'),        # cartographerパッケージから
        'rviz',                                             # rviz設定ディレクトリ
        'tb3_cartographer.rviz'                             # Cartographer専用の画面設定ファイル
    ])
    
    # RVizノードの起動設定
    # 地図、ロボット位置、LiDARデータなどがリアルタイム表示される
    rviz_node = Node(
        package='rviz2',                                   # RViz2パッケージ（ROS2版）
        executable='rviz2',                                # 実行ファイル名
        name='rviz2',                                      # このノードの名前
        arguments=['-d', rviz_config_file],                # 起動時に設定ファイルを読み込み
        parameters=[{'use_sim_time': use_sim_time}],       # シミュレーション時間を使用
        output='screen'                                    # ログをターミナルに出力
    )
    
    # 4. Auto exploration node (delayed start to ensure other nodes are ready)
    # 自動探索AI（私たちが作ったプログラム）を5秒遅延で起動
    # 遅延理由：Gazebo、SLAM、RVizが完全に準備完了してから動作開始するため
    auto_explorer_node = TimerAction(
        period=5.0,  # Wait 5 seconds before starting exploration
                     # 5秒待機（他システムの初期化完了を待つ）
        actions=[
            Node(
                package='auto_mapping_system',             # 私たちのパッケージ名
                executable='auto_explorer',                # 自動探索プログラム（setup.pyで定義）
                name='auto_mapping_explorer',              # このノードの名前
                parameters=[{'use_sim_time': use_sim_time}],  # シミュレーション時間設定
                output='screen'                            # AIの思考過程をターミナルで確認
            )
        ]
    )
    
    # 全ての起動設定をまとめて返す
    # ここに記載された順番で各システムが起動される
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time for all nodes'  # コマンドライン引数の説明
        ),
        gazebo_launch,          # 1. 物理シミュレーション環境
        cartographer_launch,    # 2. SLAM地図作成システム  
        rviz_node,              # 3. 3D可視化ツール
        auto_explorer_node      # 4. AI自動探索（5秒後開始）
    ])
