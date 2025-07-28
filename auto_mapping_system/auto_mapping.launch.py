#!/usr/bin/env python3

# ===== TurtleBot3 自動マッピングシステム 統合起動ファイル =====
# Physical AI 最終課題用のlaunchファイル
#
# このファイルの役割：
# 複数のROS2ノードを同時に起動して、完全な自動マッピングシステムを構築
# 
# 起動されるもの：
# 1. Gazebo（物理シミュレーション環境）+ TurtleBot3 House（家の環境）
# 2. Cartographer（SLAMアルゴリズム - 地図作成）
# 3. RViz（可視化ツール - 地図を見るため）
# 4. 自動探索ノード（私たちが作ったAI）
#
# 学習したこと：
# - ROS2 Launchシステムの使い方
# - 複数ノードの協調動作
# - タイミング制御（順番に起動する方法）
# =================================================================

# ROS2 Launchシステムに必要なライブラリをインポート
from launch import LaunchDescription                                    # Launchファイルの基本クラス
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction  # 起動時のアクション（引数、他ファイル読み込み、タイマー）
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution              # 設定値の取得とパス結合
from launch_ros.actions import Node                                    # ROS2ノード起動用
from launch_ros.substitutions import FindPackageShare                  # ROS2パッケージを探すため
from launch.launch_description_sources import PythonLaunchDescriptionSource              # Pythonベースのlaunchファイル読み込み

def generate_launch_description():
    """
    Launch設定を生成する関数
    
    この関数がlaunchファイルの中心部分。
    どのプログラムをどの順番で起動するかを決めている。
    
    ROS2のルール：この関数名は必ず「generate_launch_description」にする必要がある
    """
    
    # ===== シミュレーション時間の設定 =====
    # Gazeboの仮想時間とROS2の時間を同期させるための設定
    # 'true'にすることで、シミュレーションが遅くても正確に動作する
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # ===== 1. Gazebo物理シミュレーション + TurtleBot3 House環境 =====
    # Gazebo: 物理法則（重力、摩擦、衝突など）を再現するソフト
    # TurtleBot3 House: 家の中のような環境（部屋、壁、家具がある3D空間）
    # ここでロボットが実際に動き回る「舞台」を作る
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),      # turtlebot3_gazeboパッケージを探す
                'launch',                                   # その中のlaunchフォルダ
                'turtlebot3_house.launch.py'               # house環境用のlaunchファイル
            ])
        ])
    )
    
    # ===== 2. Cartographer SLAM システム =====
    # Cartographer: Googleが開発したSLAM（地図作成）アルゴリズム
    # SLAM = Simultaneous Localization and Mapping
    # = 「同時に自分の位置を知って、地図も作る」技術
    #
    # 仕組み：
    # - ロボットがLiDARで周りをスキャン
    # - 移動しながら各地点のデータを記録
    # - それらを組み合わせて一枚の地図を作成
    # - 同時に「地図上のどこにいるか」も計算
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_cartographer'), # cartographerパッケージを探す
                'launch',
                'cartographer.launch.py'                    # cartographer起動用ファイル
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()  # シミュレーション時間設定を渡す
    )
    
    # ===== 3. RViz 3D可視化ツール =====
    # RViz: ROS2で最もよく使われる3D表示ソフト
    # これで見えるもの：
    # - ロボットの現在位置（青い矢印）
    # - LiDARスキャンデータ（赤い点々）
    # - 作成中の地図（グレーの格子模様）
    # - ロボットが通った軌跡（線）
    # → つまり「ロボットの目で見た世界」がリアルタイムで見られる！
    
    # Cartographer専用の画面設定ファイルを指定
    # この設定で、地図作成に最適な画面レイアウトになる
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('turtlebot3_cartographer'),
        'rviz',                                     # rviz設定ファイルが入ったフォルダ
        'tb3_cartographer.rviz'                     # Cartographer用の画面設定
    ])
    
    rviz_node = Node(
        package='rviz2',                            # RViz2パッケージ（ROS2版のRViz）
        executable='rviz2',                         # 実行するプログラム名
        name='rviz2',                              # このノードの名前
        arguments=['-d', rviz_config_file],        # 起動時に設定ファイルを読み込む
        parameters=[{'use_sim_time': use_sim_time}], # シミュレーション時間を使う
        output='screen'                            # ログをターミナルに表示
    )
    
    # ===== 4. 自動探索AI（私たちが作ったプログラム）=====
    # 他のシステムが完全に準備できるまで5秒待ってから起動
    #
    # なぜ待つ必要があるか：
    # - Gazeboが3D世界を完全に読み込む時間が必要
    # - TurtleBot3のセンサーが準備完了するまで時間がかかる  
    # - SLAMシステムが初期化されるまで待つ必要がある
    # - 全部準備できてから動き始めないと、エラーになる可能性がある
    #
    # 5秒という時間は実験で決めた最適な値
    auto_explorer_node = TimerAction(
        period=5.0,  # 5秒間待機
        actions=[
            Node(
                package='auto_mapping_system',          # 私たちが作ったパッケージ名
                executable='auto_explorer',             # 実行ファイル名（setup.pyで設定した名前）
                name='auto_mapping_explorer',           # このノードの名前
                parameters=[{'use_sim_time': use_sim_time}], # シミュレーション時間設定
                output='screen'                         # AIの思考過程をターミナルで見る
            )
        ]
    )
    
    # ===== 全ての設定をまとめて返す =====
    # LaunchDescriptionに全ての起動設定を登録
    # ここに書いた順番で起動される
    return LaunchDescription([
        # コマンドライン引数の定義
        # 例：ros2 launch auto_mapping_system auto_mapping.launch.py use_sim_time:=false
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time for all nodes'  # この設定の説明
        ),
        
        # 起動順序（同時に開始されるもの）
        gazebo_launch,          # 1. 物理シミュレーション世界
        cartographer_launch,    # 2. 地図作成システム  
        rviz_node,             # 3. 3D表示画面
        auto_explorer_node     # 4. AI探索プログラム（5秒後）
    ])
