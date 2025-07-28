#!/usr/bin/env python3

# ===== TurtleBot3 自動マッピング探索ノード =====
# Physical AI 最終課題: AI駆動の自律探索システム
# 
# このプログラムの目的：
# - TurtleBot3が自動的に動き回って環境を探索
# - LiDARセンサーで障害物を検知しながら安全にナビゲーション
# - リアルタイムでマップを作成（SLAMアルゴリズム使用）
# - 袋小路に入ったときの脱出機能も実装
#
# 学習した技術：
# - ROS2のPublisher/Subscriber
# - センサーデータ処理（LiDAR）
# - 状態機械（State Machine）
# - 確率的ロボティクス
# =============================================

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist           # ロボットの移動指令用
from sensor_msgs.msg import LaserScan         # LiDARセンサーデータ用
from nav_msgs.msg import OccupancyGrid        # 地図データ用
import numpy as np                            # 数値計算用
import random                                 # ランダム探索用
import math                                   # 三角関数などの計算用
import time                                   # 時間計測用

class AutoMappingExplorer(Node):
    """
    自動マッピング探索クラス
    
    このクラスが行うこと：
    1. LiDARデータを受信して障害物を検知
    2. 地図データを受信して探索進捗を計算
    3. 状態機械を使って行動を決定（探索/回転/後退）
    4. ロボットに移動指令を送信
    """
    
    def __init__(self):
        super().__init__('auto_mapping_explorer')
        
        # ===== ROS2通信の設定 =====
        # Publisher: ロボットに移動指令を送信
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber: センサーデータを受信
        # LiDARデータ（360度の距離情報）を受信
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        # 地図データ（占有格子マップ）を受信
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        # ===== ロボットの状態管理 =====
        self.laser_data = None                   # LiDARデータを保存
        self.current_map = None                  # 現在の地図データを保存
        self.exploration_state = "exploring"     # 現在の行動状態（exploring, turning, backing）
        self.turn_direction = 1                  # 回転方向（1=左, -1=右）
        self.backup_time = 0                     # 後退している時間をカウント
        self.turn_time = 0                       # 回転している時間をカウント
        self.start_time = time.time()            # 探索開始時刻を記録
        
        # ===== 移動パラメータ（実験で調整した値） =====
        self.linear_speed = 0.22                 # 前進速度 [m/s]
        self.angular_speed = 0.6                 # 回転速度 [rad/s]
        self.safe_distance = 0.35                # 安全距離 [m] - これより近いと障害物とみなす
        self.wall_follow_distance = 0.45         # 壁追従距離 [m] - この距離で壁に沿って移動
        self.exploration_time = 0                # 探索開始からの経過時間をカウント
        
        # ===== AI探索用パラメータ =====
        self.last_turn_time = 0                  # 最後に回転した時刻
        self.stuck_counter = 0                   # スタック（行き詰まり）回数をカウント
        self.previous_position = None            # 前回の位置（将来の拡張用）
        self.position_history = []               # 位置履歴（将来の拡張用）
        
        # ===== メインループの設定 =====
        # 0.1秒ごと（10Hz）に探索ロジックを実行
        self.exploration_timer = self.create_timer(0.1, self.explore_callback)
        
        # 起動メッセージ（ログに出力）
        self.get_logger().info("Auto Mapping Explorer started!")
        self.get_logger().info("TurtleBot3 will now automatically explore and create a map")
        self.get_logger().info("Watch the map build in real-time in RViz!")
    
    def laser_callback(self, msg):
        """
        LiDARセンサーデータを受信する関数
        
        LiDARは360度回転しながらレーザーを照射して、
        各角度での障害物までの距離を測定する。
        この情報を使って障害物回避や壁追従を行う。
        """
        self.laser_data = msg
    
    def map_callback(self, msg):
        """
        地図データを受信して探索進捗を計算する関数
        
        地図は占有格子マップという形式で、各セルが：
        - -1: 未探索エリア（まだ行ったことがない場所）
        - 0-50: 自由空間（通れる場所）
        - 51-100: 障害物（壁や家具など）
        """
        self.current_map = msg
        
        # 地図の探索進捗を計算（どのくらい探索できたかのパーセンテージ）
        if msg.data:
            total_cells = len(msg.data)                                   # 地図の総セル数
            unknown_cells = sum(1 for cell in msg.data if cell == -1)     # 未探索セル数
            obstacle_cells = sum(1 for cell in msg.data if cell > 50)     # 障害物セル数
            free_cells = sum(1 for cell in msg.data if cell >= 0 and cell <= 50)  # 自由空間セル数
            
            if total_cells > 0:
                coverage = (free_cells / total_cells) * 100                # 自由空間の割合
                exploration_ratio = ((total_cells - unknown_cells) / total_cells) * 100  # 探索済みの割合
                
                # 10秒ごとに進捗をログ出力（100カウント = 10秒、0.1秒×100）
                if self.exploration_time % 100 == 0:
                    elapsed_time = time.time() - self.start_time
                    self.get_logger().info(
                        f"Exploration Progress: {exploration_ratio:.1f}% | "
                        f"Map Coverage: {coverage:.1f}% | "
                        f"Time: {elapsed_time:.1f}s"
                    )
    
    def explore_callback(self):
        """
        メイン探索ロジック
        
        この関数が0.1秒ごとに呼ばれて、センサーデータを分析し、
        次の行動を決定する。ロボットの思考部分。
        
        処理の流れ：
        1. LiDARデータから各方向の距離を計算
        2. 現在の状態に基づいて行動を決定
        3. 移動指令をロボットに送信
        """
        # LiDARデータがまだ届いていない場合は何もしない
        if self.laser_data is None:
            return
        
        self.exploration_time += 1  # 経過時間をカウントアップ
        cmd = Twist()               # 移動指令を初期化
        
        # ===== LiDARデータの前処理 =====
        # NumPy配列に変換し、無限大や無効な値を除去
        ranges = np.array(self.laser_data.ranges)
        ranges = ranges[np.isfinite(ranges)]  # 有限の値のみを残す
        
        if len(ranges) == 0:
            return  # 有効なデータがない場合は何もしない
        
        # ===== 各方向の距離を計算 =====
        # ロボットの周りを複数のセクターに分けて、それぞれの最小距離を取得
        front_distance = self.get_min_distance_in_sector(ranges, -0.4, 0.4)      # 前方
        left_distance = self.get_min_distance_in_sector(ranges, 0.4, 1.6)        # 左側
        right_distance = self.get_min_distance_in_sector(ranges, -1.6, -0.4)     # 右側
        left_front = self.get_min_distance_in_sector(ranges, 0.2, 0.8)           # 左前方
        right_front = self.get_min_distance_in_sector(ranges, -0.8, -0.2)        # 右前方
        
        # ===== 状態機械による行動決定 =====
        # 状態機械：ロボットの行動を「状態」で管理する設計パターン
        # 現在の状態に応じて異なる行動を取る
        
        if self.exploration_state == "exploring":
            # 【探索状態】通常の探索行動
            
            if front_distance < self.safe_distance:
                # 前方に障害物発見！回転が必要
                # より空間がある方向に回転する（左右どちらが空いているか判断）
                if left_distance > right_distance:
                    self.turn_direction = 1  # 左回転
                else:
                    self.turn_direction = -1  # 右回転
                
                # 状態を「回転中」に変更
                self.exploration_state = "turning"
                self.turn_time = 0
                self.last_turn_time = self.exploration_time
                self.get_logger().info(f"🚧 Obstacle ahead! Turning {'left' if self.turn_direction > 0 else 'right'}")
            
            else:
                # 前方クリア！探索を続行
                cmd.linear.x = self.linear_speed  # 前進する
                
                # ===== 壁追従アルゴリズム =====
                # 壁に沿って移動することで効率的に探索する
                if left_distance < self.wall_follow_distance and front_distance > self.safe_distance:
                    # 左壁追従モード
                    if left_distance < 0.25:
                        cmd.angular.z = -0.3  # 壁から離れるように右に曲がる
                    elif left_distance > 0.5:
                        cmd.angular.z = 0.2   # 壁に近づくように左に曲がる
                    self.get_logger().info("Following left wall", throttle_duration_sec=2.0)
                
                elif right_distance < self.wall_follow_distance and front_distance > self.safe_distance:
                    # 右壁追従モード
                    if right_distance < 0.25:
                        cmd.angular.z = 0.3   # 壁から離れるように左に曲がる
                    elif right_distance > 0.5:
                        cmd.angular.z = -0.2  # 壁に近づくように右に曲がる
                    self.get_logger().info("Following right wall", throttle_duration_sec=2.0)
                
                else:
                    # ===== オープンスペース探索 =====
                    # 壁がない広い空間での探索戦略
                    if self.exploration_time % 150 == 0:  # 15秒ごとに方向変更
                        self.turn_direction = random.choice([-1, 1])  # ランダムに左右を選択
                        cmd.angular.z = self.turn_direction * 0.3
                        self.get_logger().info("Random exploration turn")
                    
                    # ループ防止のための軽微な方向調整
                    if random.random() < 0.05:  # 5%の確率で
                        cmd.angular.z = self.turn_direction * 0.2
        
        elif self.exploration_state == "turning":
            # 【回転状態】障害物を避けるために回転中
            cmd.angular.z = self.turn_direction * self.angular_speed
            self.turn_time += 1
            
            # 回転完了の判定
            if front_distance > self.safe_distance * 1.8 or self.turn_time > 40:
                self.exploration_state = "exploring"  # 探索状態に戻る
                self.get_logger().info("Turn complete, resuming exploration")
        
        elif self.exploration_state == "backing":
            # 【後退状態】行き詰まったときの脱出行動
            cmd.linear.x = -0.15                          # 後退
            cmd.angular.z = self.turn_direction * 0.4     # 回転しながら後退
            self.backup_time += 1
            
            # 後退完了の判定
            if self.backup_time > 25:
                self.exploration_state = "turning"  # 回転状態に移行
                self.backup_time = 0
                self.turn_time = 0
                self.get_logger().info("Backup complete, starting turn")
        
        # ===== スタック検出と回復 =====
        # ロボットが行き詰まった状況を検出して自動回復
        if self.detect_stuck_situation(front_distance, left_distance, right_distance):
            if self.exploration_state != "backing":
                self.exploration_state = "backing"  # 後退状態に強制移行
                self.backup_time = 0
                self.stuck_counter += 1
                self.get_logger().info(f"Stuck situation detected (#{self.stuck_counter}), initiating recovery")
        
        # ===== 高度な障害物回避 =====
        # 基本的な行動決定の後に、緊急回避処理を適用
        if self.laser_data is not None:
            cmd = self.advanced_obstacle_avoidance(cmd, ranges)
        
        # ===== 安全制限の適用 =====
        # 速度が危険な値にならないように制限
        cmd.linear.x = max(-0.2, min(0.3, cmd.linear.x))    # 前進: 0-0.3 m/s, 後退: 0-0.2 m/s
        cmd.angular.z = max(-1.0, min(1.0, cmd.angular.z))  # 回転: ±1.0 rad/s
        
        # ===== 移動指令の送信 =====
        # 計算した移動指令をロボットに送信
        self.cmd_pub.publish(cmd)
    
    def get_min_distance_in_sector(self, ranges, start_angle, end_angle):
        """
        指定した角度範囲での最小距離を取得する関数
        
        LiDARは360度の距離データを配列で提供するので、
        特定の方向（前方、左、右など）の距離を知るには
        該当する角度範囲のデータを抽出する必要がある。
        
        引数:
            ranges: LiDARの距離データ配列
            start_angle: 開始角度 [ラジアン]
            end_angle: 終了角度 [ラジアン]
        
        戻り値:
            その角度範囲での最小距離 [m]
        """
        total_angles = len(ranges)
        angle_increment = 2 * math.pi / total_angles  # 各データ点の角度間隔
        
        # 角度をインデックスに変換
        start_idx = int((start_angle + math.pi) / angle_increment)
        end_idx = int((end_angle + math.pi) / angle_increment)
        
        # インデックスが配列の範囲内になるように調整
        start_idx = max(0, min(start_idx, total_angles - 1))
        end_idx = max(0, min(end_idx, total_angles - 1))
        
        if start_idx > end_idx:
            # 角度範囲が0度をまたぐ場合（例：-30度から30度）
            sector_ranges = np.concatenate([ranges[start_idx:], ranges[:end_idx+1]])
        else:
            # 通常の場合
            sector_ranges = ranges[start_idx:end_idx+1]
        
        return np.min(sector_ranges) if len(sector_ranges) > 0 else float('inf')
    
    def detect_stuck_situation(self, front_dist, left_dist, right_dist):
        """
        ロボットが行き詰まった状況を検出する関数
        
        以下の状況をスタック（行き詰まり）と判定：
        1. 四方が狭い空間に囲まれている
        2. 正面に非常に近い障害物がある
        3. 最近回転していない状態で正面が塞がれている
        
        これらを検出することで、袋小路や狭い場所での
        自動脱出が可能になる。
        """
        # 四方八方が狭い状況
        surrounded = (front_dist < 0.25 and left_dist < 0.25 and right_dist < 0.25)
        
        # 正面に非常に近い障害物
        very_close_front = front_dist < 0.15
        
        # 最近回転していない（50カウント = 5秒間）
        recent_turn = (self.exploration_time - self.last_turn_time) < 50
        
        return surrounded or (very_close_front and not recent_turn)
    
    def advanced_obstacle_avoidance(self, cmd, ranges):
        """
        高度な障害物回避処理
        
        基本的な探索ロジックに加えて、緊急時の回避行動を実装。
        非常に近い障害物を検出した場合、即座に回避行動を取る。
        
        これにより、基本ロジックだけでは対応できない
        急な障害物に対しても安全にナビゲーションできる。
        """
        close_obstacle_threshold = 0.2  # 緊急回避する距離の閾値
        
        # より細かく前方をチェック
        front_left = self.get_min_distance_in_sector(ranges, 0.1, 0.5)      # 左前方
        front_right = self.get_min_distance_in_sector(ranges, -0.5, -0.1)   # 右前方
        immediate_front = self.get_min_distance_in_sector(ranges, -0.1, 0.1) # 真正面
        
        if immediate_front < close_obstacle_threshold:
            # 真正面に緊急障害物！即座に停止して回転
            cmd.linear.x = 0.0  # 停止
            if front_left > front_right:
                cmd.angular.z = 0.8   # 左に素早く回転
            else:
                cmd.angular.z = -0.8  # 右に素早く回転
        
        elif front_left < close_obstacle_threshold:
            # 左前方に緊急障害物！右に回避
            cmd.angular.z = -0.5
        
        elif front_right < close_obstacle_threshold:
            # 右前方に緊急障害物！左に回避
            cmd.angular.z = 0.5
        
        return cmd

def main(args=None):
    """
    メイン関数：プログラムのエントリーポイント
    
    ROS2ノードを初期化し、探索ノードを起動する。
    Ctrl+Cで停止された場合は、安全にロボットを停止する。
    """
    rclpy.init(args=args)
    explorer = AutoMappingExplorer()
    
    try:
        # ノードを実行（無限ループで各コールバック関数が呼ばれる）
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        # Ctrl+Cで停止された場合の処理
        cmd = Twist()  # 停止指令を作成
        explorer.cmd_pub.publish(cmd)  # ロボットを停止
        explorer.get_logger().info("Auto mapping stopped by user")
        explorer.get_logger().info("Final map has been generated!")
    finally:
        # 終了処理
        explorer.destroy_node()
        rclpy.shutdown()

# Pythonスクリプトとして直接実行された場合にmain関数を呼び出し
if __name__ == '__main__':
    main()
