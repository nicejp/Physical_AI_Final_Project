#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import random
import math
import time

class AutoMappingExplorer(Node):
    def __init__(self):
        super().__init__('auto_mapping_explorer')
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        # State variables
        self.laser_data = None
        self.current_map = None
        self.exploration_state = "exploring"  # exploring, turning, backing
        self.turn_direction = 1  # 1 for left, -1 for right
        self.backup_time = 0
        self.turn_time = 0
        self.start_time = time.time()
        
        # Movement parameters
        self.linear_speed = 0.22
        self.angular_speed = 0.6
        self.safe_distance = 0.35
        self.wall_follow_distance = 0.45
        self.exploration_time = 0
        
        # AI exploration parameters
        self.last_turn_time = 0
        self.stuck_counter = 0
        self.previous_position = None
        self.position_history = []
        
        # Timer for continuous exploration
        self.exploration_timer = self.create_timer(0.1, self.explore_callback)
        
        self.get_logger().info("Auto Mapping Explorer started!")
        self.get_logger().info("TurtleBot3 will now automatically explore and create a map")
        self.get_logger().info("Watch the map build in real-time in RViz!")
    
    def laser_callback(self, msg):
        """Process laser scan data for obstacle detection and navigation"""
        self.laser_data = msg
    
    def map_callback(self, msg):
        """Process map data and calculate exploration progress"""
        self.current_map = msg
        
        # Calculate map coverage for progress tracking
        if msg.data:
            total_cells = len(msg.data)
            unknown_cells = sum(1 for cell in msg.data if cell == -1)
            obstacle_cells = sum(1 for cell in msg.data if cell > 50)
            free_cells = sum(1 for cell in msg.data if cell >= 0 and cell <= 50)
            
            if total_cells > 0:
                coverage = (free_cells / total_cells) * 100
                exploration_ratio = ((total_cells - unknown_cells) / total_cells) * 100
                
                # Log progress every 10 seconds
                if self.exploration_time % 100 == 0:
                    elapsed_time = time.time() - self.start_time
                    self.get_logger().info(
                        f"Exploration Progress: {exploration_ratio:.1f}% | "
                        f"Map Coverage: {coverage:.1f}% | "
                        f"Time: {elapsed_time:.1f}s"
                    )
    
    def explore_callback(self):
        """Main exploration logic with AI-driven decision making"""
        if self.laser_data is None:
            return
        
        self.exploration_time += 1
        cmd = Twist()
        
        # Process laser scan data
        ranges = np.array(self.laser_data.ranges)
        ranges = ranges[np.isfinite(ranges)]
        
        if len(ranges) == 0:
            return
        
        # Calculate distances in different sectors
        front_distance = self.get_min_distance_in_sector(ranges, -0.4, 0.4)
        left_distance = self.get_min_distance_in_sector(ranges, 0.4, 1.6)
        right_distance = self.get_min_distance_in_sector(ranges, -1.6, -0.4)
        left_front = self.get_min_distance_in_sector(ranges, 0.2, 0.8)
        right_front = self.get_min_distance_in_sector(ranges, -0.8, -0.2)
        
        # Advanced exploration state machine
        if self.exploration_state == "exploring":
            if front_distance < self.safe_distance:
                # Decide turn direction based on space available
                if left_distance > right_distance:
                    self.turn_direction = 1  # Turn left
                else:
                    self.turn_direction = -1  # Turn right
                
                self.exploration_state = "turning"
                self.turn_time = 0
                self.last_turn_time = self.exploration_time
                self.get_logger().info(f"🚧 Obstacle ahead! Turning {'left' if self.turn_direction > 0 else 'right'}")
            
            else:
                # Free space exploration with intelligent behavior
                cmd.linear.x = self.linear_speed
                
                # Wall following behavior for better coverage
                if left_distance < self.wall_follow_distance and front_distance > self.safe_distance:
                    # Follow left wall
                    if left_distance < 0.25:
                        cmd.angular.z = -0.3  # Turn away from wall
                    elif left_distance > 0.5:
                        cmd.angular.z = 0.2   # Turn toward wall
                    self.get_logger().info("Following left wall", throttle_duration_sec=2.0)
                
                elif right_distance < self.wall_follow_distance and front_distance > self.safe_distance:
                    # Follow right wall
                    if right_distance < 0.25:
                        cmd.angular.z = 0.3   # Turn away from wall
                    elif right_distance > 0.5:
                        cmd.angular.z = -0.2  # Turn toward wall
                    self.get_logger().info("Following right wall", throttle_duration_sec=2.0)
                
                else:
                    # Open space exploration with randomness
                    if self.exploration_time % 150 == 0:  # Change direction periodically
                        self.turn_direction = random.choice([-1, 1])
                        cmd.angular.z = self.turn_direction * 0.3
                        self.get_logger().info("Random exploration turn")
                    
                    # Slight bias to prevent loops
                    if random.random() < 0.05:  # 5% chance
                        cmd.angular.z = self.turn_direction * 0.2
        
        elif self.exploration_state == "turning":
            # Execute turn maneuver
            cmd.angular.z = self.turn_direction * self.angular_speed
            self.turn_time += 1
            
            # Check if turn is complete
            if front_distance > self.safe_distance * 1.8 or self.turn_time > 40:
                self.exploration_state = "exploring"
                self.get_logger().info("Turn complete, resuming exploration")
        
        elif self.exploration_state == "backing":
            # Backup maneuver when stuck
            cmd.linear.x = -0.15
            cmd.angular.z = self.turn_direction * 0.4
            self.backup_time += 1
            
            if self.backup_time > 25:
                self.exploration_state = "turning"
                self.backup_time = 0
                self.turn_time = 0
                self.get_logger().info("Backup complete, starting turn")
        
        # Stuck detection and recovery
        if self.detect_stuck_situation(front_distance, left_distance, right_distance):
            if self.exploration_state != "backing":
                self.exploration_state = "backing"
                self.backup_time = 0
                self.stuck_counter += 1
                self.get_logger().info(f"Stuck situation detected (#{self.stuck_counter}), initiating recovery")
        
        # Advanced obstacle avoidance
        if self.laser_data is not None:
            cmd = self.advanced_obstacle_avoidance(cmd, ranges)
        
        # Safety limits
        cmd.linear.x = max(-0.2, min(0.3, cmd.linear.x))
        cmd.angular.z = max(-1.0, min(1.0, cmd.angular.z))
        
        # Publish movement command
        self.cmd_pub.publish(cmd)
    
    def get_min_distance_in_sector(self, ranges, start_angle, end_angle):
        """Get minimum distance in a specific angular sector"""
        total_angles = len(ranges)
        angle_increment = 2 * math.pi / total_angles
        
        start_idx = int((start_angle + math.pi) / angle_increment)
        end_idx = int((end_angle + math.pi) / angle_increment)
        
        start_idx = max(0, min(start_idx, total_angles - 1))
        end_idx = max(0, min(end_idx, total_angles - 1))
        
        if start_idx > end_idx:
            # Handle wrap-around
            sector_ranges = np.concatenate([ranges[start_idx:], ranges[:end_idx+1]])
        else:
            sector_ranges = ranges[start_idx:end_idx+1]
        
        return np.min(sector_ranges) if len(sector_ranges) > 0 else float('inf')
    
    def detect_stuck_situation(self, front_dist, left_dist, right_dist):
        """Detect if robot is stuck in a corner or tight space"""
        # Multiple criteria for stuck detection
        surrounded = (front_dist < 0.25 and left_dist < 0.25 and right_dist < 0.25)
        very_close_front = front_dist < 0.15
        recent_turn = (self.exploration_time - self.last_turn_time) < 50
        
        return surrounded or (very_close_front and not recent_turn)
    
    def advanced_obstacle_avoidance(self, cmd, ranges):
        """Advanced obstacle avoidance using full laser scan"""
        # Check for very close obstacles and react immediately
        close_obstacle_threshold = 0.2
        
        # Front sectors
        front_left = self.get_min_distance_in_sector(ranges, 0.1, 0.5)
        front_right = self.get_min_distance_in_sector(ranges, -0.5, -0.1)
        immediate_front = self.get_min_distance_in_sector(ranges, -0.1, 0.1)
        
        if immediate_front < close_obstacle_threshold:
            cmd.linear.x = 0.0
            if front_left > front_right:
                cmd.angular.z = 0.8  # Turn left quickly
            else:
                cmd.angular.z = -0.8  # Turn right quickly
        
        elif front_left < close_obstacle_threshold:
            cmd.angular.z = -0.5  # Turn right to avoid left obstacle
        
        elif front_right < close_obstacle_threshold:
            cmd.angular.z = 0.5   # Turn left to avoid right obstacle
        
        return cmd

def main(args=None):
    rclpy.init(args=args)
    explorer = AutoMappingExplorer()
    
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        # Graceful shutdown
        cmd = Twist()
        explorer.cmd_pub.publish(cmd)
        explorer.get_logger().info("Auto mapping stopped by user")
        explorer.get_logger().info("Final map has been generated!")
    finally:
        explorer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
