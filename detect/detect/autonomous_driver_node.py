#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Odometry
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


class AutonomousDriverNode(Node):
    def __init__(self):
        super().__init__('autonomous_driver')
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # Goal değişkenleri
        self.goal_x = None
        self.goal_y = None
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
        
        # Encoder subscription for position tracking
        from nav_msgs.msg import Odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/get_encoder/odom',
            self.odom_callback,
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
        
        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        self.status_timer = self.create_timer(1.0, self.publish_status)  # 1 Hz
        
        # Manuel komut
        self.manual_cmd = Twist()
        
        self.get_logger().info("Otonom Sürücü Node başlatıldı")
        self.get_logger().info("Mevcut mod: MANUAL")
    def goal_callback(self, msg):
        """Hedef pozisyon callback"""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.get_logger().info(f"Yeni hedef alındı: ({self.goal_x:.2f}, {self.goal_y:.2f})")
    
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

    def odom_callback(self, msg):
        """Odometry callback - robot pozisyonunu güncelle"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Quaternion'dan yaw açısını hesapla - basit yöntem
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        """Ana kontrol döngüsü"""
        if self.emergency_stop:
            self.stop_robot()
            return
        
        if self.current_mode == DrivingMode.MANUAL:
            self.handle_manual_mode()
        elif self.current_mode == DrivingMode.AUTONOMOUS:
            self.handle_autonomous_mode()
        elif self.current_mode == DrivingMode.EMERGENCY_STOP:
            self.stop_robot()

    def handle_manual_mode(self):
        """Manuel mod kontrolü"""
        # Manuel komutları doğrudan ilet
        self.cmd_vel_pub.publish(self.manual_cmd)

    def handle_autonomous_mode(self):
        """Otonom mod kontrolü"""
        # Hedefe ulaştık mı kontrol et
        if self.goal_x is not None and self.goal_y is not None:
            distance_to_goal = math.sqrt((self.goal_x - self.robot_x)**2 + (self.goal_y - self.robot_y)**2)
            if distance_to_goal < 0.5:  # Hedefe vardık
                self.stop_robot()
                self.get_logger().info("Hedefe ulaşıldı!")
                return
        
        # Yol açık mı kontrol et
        if not self.is_path_clear():
            # Basit engel kaçınma
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.2
            cmd_vel.angular.z = 0.5  # Sola dön
            self.cmd_vel_pub.publish(cmd_vel)
        else:
            # Hedefe doğru git
            if self.goal_x is not None and self.goal_y is not None:
                # Hedefe yönelim hesapla
                target_angle = math.atan2(self.goal_y - self.robot_y, self.goal_x - self.robot_x)
                angle_diff = target_angle - self.robot_yaw
                
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.5
                cmd_vel.angular.z = angle_diff * 0.5  # Açı düzeltmesi
                self.cmd_vel_pub.publish(cmd_vel)
            else:
                # Hedef yok, dur
                self.stop_robot()

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
            "path_clear": self.is_path_clear()
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
