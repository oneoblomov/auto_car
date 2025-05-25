#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
import numpy as np
import math
import json
from enum import Enum


class DrivingMode(Enum):
    MANUAL = "manual"
    AUTONOMOUS = "autonomous"
    EMERGENCY_STOP = "emergency_stop"
    FOLLOW_WALL = "follow_wall"
    EXPLORE = "explore"


class AutonomousDriverNode(Node):
    def __init__(self):
        super().__init__('autonomous_driver')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.status_pub = self.create_publisher(String, '/autonomous_status', 10)
        
        # Subscribers
        self.mode_sub = self.create_subscription(
            String,
            '/driving_mode',
            self.mode_callback,
            10
        )
        
        self.emergency_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.emergency_callback,
            10
        )
        
        self.obstacle_map_sub = self.create_subscription(
            OccupancyGrid,
            '/detect_obstacle/obstacle_map',
            self.obstacle_map_callback,
            10
        )
        
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/get_lidar/scan',
            self.lidar_callback,
            10
        )
        
        self.manual_cmd_sub = self.create_subscription(
            Twist,
            '/manual_cmd_vel',
            self.manual_cmd_callback,
            10
        )
        
        # Durum değişkenleri
        self.current_mode = DrivingMode.MANUAL
        self.emergency_stop = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # Sensor verileri
        self.latest_lidar = None
        self.obstacle_map = None
        
        # Otonom sürüş parametreleri
        self.max_linear_speed = 1.5  # m/s
        self.max_angular_speed = 1.0  # rad/s
        self.safe_distance = 1.0  # metre
        self.wall_follow_distance = 0.8  # metre
        
        # Keşif modu için
        self.exploration_goals = []
        self.current_goal_index = 0
        
        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        self.status_timer = self.create_timer(1.0, self.publish_status)  # 1 Hz
        
        # Manuel komut
        self.manual_cmd = Twist()
        
        self.get_logger().info("Otonom Sürücü Node başlatıldı")
        self.get_logger().info("Mevcut mod: MANUAL")

    def mode_callback(self, msg):
        """Sürüş modu değiştirme"""
        try:
            new_mode = DrivingMode(msg.data.lower())
            old_mode = self.current_mode
            self.current_mode = new_mode
            
            self.get_logger().info(f"Sürüş modu değişti: {old_mode.value} -> {new_mode.value}")
            
            # Mod geçiş işlemleri
            if new_mode == DrivingMode.AUTONOMOUS:
                self.start_autonomous_mode()
            elif new_mode == DrivingMode.FOLLOW_WALL:
                self.start_wall_follow_mode()
            elif new_mode == DrivingMode.EXPLORE:
                self.start_exploration_mode()
            elif new_mode == DrivingMode.MANUAL:
                self.stop_robot()
                
        except ValueError:
            self.get_logger().error(f"Geçersiz sürüş modu: {msg.data}")

    def emergency_callback(self, msg):
        """Acil durum callback"""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.current_mode = DrivingMode.EMERGENCY_STOP
            self.stop_robot()
            self.get_logger().warn("ACİL DURUM AKTIF!")
        else:
            self.get_logger().info("Acil durum sıfırlandı")

    def obstacle_map_callback(self, msg):
        """Engel haritası callback"""
        self.obstacle_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    def lidar_callback(self, msg):
        """LiDAR callback"""
        self.latest_lidar = msg

    def manual_cmd_callback(self, msg):
        """Manuel komut callback"""
        self.manual_cmd = msg

    def control_loop(self):
        """Ana kontrol döngüsü"""
        if self.emergency_stop:
            self.stop_robot()
            return
        
        if self.current_mode == DrivingMode.MANUAL:
            self.handle_manual_mode()
        elif self.current_mode == DrivingMode.AUTONOMOUS:
            self.handle_autonomous_mode()
        elif self.current_mode == DrivingMode.FOLLOW_WALL:
            self.handle_wall_follow_mode()
        elif self.current_mode == DrivingMode.EXPLORE:
            self.handle_exploration_mode()
        elif self.current_mode == DrivingMode.EMERGENCY_STOP:
            self.stop_robot()

    def handle_manual_mode(self):
        """Manuel mod kontrolü"""
        # Manuel komutları doğrudan ilet
        self.cmd_vel_pub.publish(self.manual_cmd)

    def handle_autonomous_mode(self):
        """Otonom mod kontrolü"""
        # Path planner'dan gelen komutları kullan
        # Bu mod path_planner_node ile koordine çalışır
        pass

    def handle_wall_follow_mode(self):
        """Duvar takip modu"""
        if self.latest_lidar is None:
            return
        
        cmd_vel = Twist()
        
        # Sağ duvara göre takip et
        right_distances = []
        front_distances = []
        
        # LiDAR verilerini analiz et
        for i, range_val in enumerate(self.latest_lidar.ranges):
            if (range_val < self.latest_lidar.range_min or 
                range_val > self.latest_lidar.range_max):
                continue
            
            angle = self.latest_lidar.angle_min + i * self.latest_lidar.angle_increment
            
            # Sağ taraf (yaklaşık -90 derece)
            if -math.pi/2 - 0.3 < angle < -math.pi/2 + 0.3:
                right_distances.append(range_val)
            
            # Ön taraf (yaklaşık 0 derece)
            if -0.3 < angle < 0.3:
                front_distances.append(range_val)
        
        # Ortalama mesafeleri hesapla
        avg_right_distance = np.mean(right_distances) if right_distances else float('inf')
        avg_front_distance = np.mean(front_distances) if front_distances else float('inf')
        
        # Duvar takip algoritması
        if avg_front_distance < self.safe_distance:
            # Önde engel var, sola dön
            cmd_vel.angular.z = 0.5
            cmd_vel.linear.x = 0.2
        elif avg_right_distance > self.wall_follow_distance + 0.2:
            # Duvardan uzaklaştık, sağa dön
            cmd_vel.angular.z = -0.3
            cmd_vel.linear.x = 0.5
        elif avg_right_distance < self.wall_follow_distance - 0.2:
            # Duvara çok yaklaştık, sola dön
            cmd_vel.angular.z = 0.3
            cmd_vel.linear.x = 0.5
        else:
            # Düz git
            cmd_vel.linear.x = 0.8
            cmd_vel.angular.z = 0.0
        
        # Hız limitlerini uygula
        cmd_vel.linear.x = max(0.0, min(cmd_vel.linear.x, self.max_linear_speed))
        cmd_vel.angular.z = max(-self.max_angular_speed, min(cmd_vel.angular.z, self.max_angular_speed))
        
        self.cmd_vel_pub.publish(cmd_vel)

    def handle_exploration_mode(self):
        """Keşif modu"""
        if not self.exploration_goals:
            self.generate_exploration_goals()
        
        # Mevcut hedefe git
        if self.current_goal_index < len(self.exploration_goals):
            goal = self.exploration_goals[self.current_goal_index]
            
            # Hedefe ulaştık mı kontrol et
            distance = math.sqrt((goal[0] - self.robot_x)**2 + (goal[1] - self.robot_y)**2)
            if distance < 1.0:  # 1 metre yakınsa
                self.current_goal_index += 1
                self.get_logger().info(f"Keşif hedefi {self.current_goal_index}/{len(self.exploration_goals)} tamamlandı")
            
            # Hedef yayınla
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'
            goal_msg.pose.position.x = goal[0]
            goal_msg.pose.position.y = goal[1]
            goal_msg.pose.orientation.w = 1.0
            
            self.goal_pub.publish(goal_msg)
        else:
            # Tüm hedefler tamamlandı, yenilerini oluştur
            self.exploration_goals = []
            self.current_goal_index = 0

    def start_autonomous_mode(self):
        """Otonom modu başlat"""
        self.get_logger().info("Otonom mod başlatıldı")
        # Basit bir hedef belirle (ileride kullanıcıdan alınabilir)
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = 5.0  # 5 metre ileri
        goal_msg.pose.position.y = 0.0
        goal_msg.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal_msg)

    def start_wall_follow_mode(self):
        """Duvar takip modunu başlat"""
        self.get_logger().info("Duvar takip modu başlatıldı")

    def start_exploration_mode(self):
        """Keşif modunu başlat"""
        self.get_logger().info("Keşif modu başlatıldı")
        self.exploration_goals = []
        self.current_goal_index = 0

    def generate_exploration_goals(self):
        """Keşif hedefleri oluştur"""
        # Basit bir keşif paterni - kare şeklinde
        goals = [
            [3.0, 0.0],   # İleri
            [3.0, 3.0],   # Sağa
            [0.0, 3.0],   # Geri
            [0.0, 0.0],   # Başlangıç
            [-3.0, 0.0],  # Sol
            [-3.0, -3.0], # Arka sol
            [0.0, -3.0],  # Arka
            [0.0, 0.0]    # Başlangıç
        ]
        
        self.exploration_goals = goals
        self.get_logger().info(f"{len(goals)} keşif hedefi oluşturuldu")

    def stop_robot(self):
        """Robotu durdur"""
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

    def is_path_clear(self, distance: float = 2.0) -> bool:
        """Önde yol açık mı kontrol et"""
        if self.latest_lidar is None:
            return False
        
        # Ön bölgeyi kontrol et
        for i, range_val in enumerate(self.latest_lidar.ranges):
            if (range_val < self.latest_lidar.range_min or 
                range_val > self.latest_lidar.range_max):
                continue
            
            angle = self.latest_lidar.angle_min + i * self.latest_lidar.angle_increment
            
            # Ön 60 derece alanını kontrol et
            if -math.pi/6 < angle < math.pi/6:
                if range_val < distance:
                    return False
        
        return True

    def publish_status(self):
        """Durum bilgisi yayınla"""
        status = {
            "mode": self.current_mode.value,
            "emergency_stop": self.emergency_stop,
            "position": {"x": self.robot_x, "y": self.robot_y, "yaw": self.robot_yaw},
            "path_clear": self.is_path_clear(),
            "exploration_progress": f"{self.current_goal_index}/{len(self.exploration_goals)}" if self.exploration_goals else "0/0"
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = AutonomousDriverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Otonom Sürücü Node durduruluyor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
