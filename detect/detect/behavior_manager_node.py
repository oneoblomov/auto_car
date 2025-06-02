#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
import numpy as np
import math
import json
from enum import Enum


class BehaviorState(Enum):
    IDLE = "idle"
    MOVING = "moving"
    AVOIDING_OBSTACLE = "avoiding_obstacle"
    STUCK = "stuck"
    TURNING = "turning"
    EMERGENCY = "emergency"


class BehaviorManagerNode(Node):
    def __init__(self):
        super().__init__('behavior_manager')
        
        # Publishers
        self.behavior_cmd_pub = self.create_publisher(Twist, '/behavior_cmd_vel', 10)
        self.emergency_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.behavior_status_pub = self.create_publisher(String, '/behavior_status', 10)
        
        # Subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/get_lidar/scan',
            self.lidar_callback,
            10
        )
        
        self.input_cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.input_cmd_callback,
            10
        )
        
        # Davranış durumu
        self.current_state = BehaviorState.IDLE
        self.previous_state = BehaviorState.IDLE
        self.state_start_time = self.get_clock().now()
        
        # Sensor verileri
        self.latest_lidar = None
        self.input_cmd = Twist()
        
        # Davranış parametreleri - Daha hassas ayarlar
        self.emergency_distance = 0.4  # metre (artırıldı)
        self.obstacle_distance = 1.2  # metre (artırıldı)
        self.stuck_threshold = 0.1  # m/s
        self.stuck_time_threshold = 2.0  # saniye (kısaltıldı)
        self.turn_duration = 1.5  # saniye (kısaltıldı)
        
        # Durum izleme
        self.last_positions = []
        self.max_position_history = 10
        self.obstacle_avoid_start_time = None
        self.turn_start_time = None
        self.last_movement_time = self.get_clock().now()
        
        # Timer
        self.timer = self.create_timer(0.1, self.behavior_loop)  # 10 Hz
        self.status_timer = self.create_timer(1.0, self.publish_status)  # 1 Hz
        
        self.get_logger().info("Davranış Yöneticisi başlatıldı")

    def lidar_callback(self, msg):
        """LiDAR callback"""
        self.latest_lidar = msg

    def input_cmd_callback(self, msg):
        """Giriş komutu callback"""
        self.input_cmd = msg

    def behavior_loop(self):
        """Ana davranış döngüsü"""
        if self.latest_lidar is None:
            return
        
        # Sensör analizleri
        distances = self.analyze_lidar()
        
        # Durum makinesini çalıştır
        self.update_behavior_state(distances)
        
        # Davranışa göre komut oluştur
        output_cmd = self.generate_behavior_command(distances)
        
        # Komutu yayınla
        self.behavior_cmd_pub.publish(output_cmd)

    def analyze_lidar(self) -> dict:
        """LiDAR verilerini analiz et"""
        if self.latest_lidar is None:
            return {}
        
        distances = {
            'front': [],
            'left': [],
            'right': [],
            'back': [],
            'front_left': [],
            'front_right': [],
            'min_distance': float('inf'),
            'emergency_zone': False
        }
        
        for i, range_val in enumerate(self.latest_lidar.ranges):
            if (range_val < self.latest_lidar.range_min or 
                range_val > self.latest_lidar.range_max):
                continue
            
            angle = self.latest_lidar.angle_min + i * self.latest_lidar.angle_increment
            
            # Açı bölgelerine göre grupla
            if -math.pi/6 < angle < math.pi/6:  # Ön
                distances['front'].append(range_val)
            elif math.pi/6 <= angle < math.pi/3:  # Ön sol
                distances['front_left'].append(range_val)
            elif math.pi/3 <= angle < 2*math.pi/3:  # Sol
                distances['left'].append(range_val)
            elif -math.pi/3 <= angle < -math.pi/6:  # Ön sağ
                distances['front_right'].append(range_val)
            elif -2*math.pi/3 <= angle < -math.pi/3:  # Sağ
                distances['right'].append(range_val)
            else:  # Arka
                distances['back'].append(range_val)
            
            # Minimum mesafeyi güncelle
            distances['min_distance'] = min(distances['min_distance'], range_val)
            
            # Acil durum bölgesi kontrolü
            if range_val < self.emergency_distance:
                distances['emergency_zone'] = True
        
        # Ortalama mesafeleri hesapla
        for key in ['front', 'left', 'right', 'back', 'front_left', 'front_right']:
            if distances[key]:
                distances[key] = np.mean(distances[key])
            else:
                distances[key] = float('inf')
        
        return distances

    def update_behavior_state(self, distances: dict):
        """Davranış durumunu güncelle"""
        current_time = self.get_clock().now()
        
        # Acil durum kontrolü
        if distances['emergency_zone']:
            self.change_state(BehaviorState.EMERGENCY)
            return
        
        # Sıkışma kontrolü
        if self.is_stuck():
            self.change_state(BehaviorState.STUCK)
            return
        
        # Mevcut duruma göre geçişler
        if self.current_state == BehaviorState.IDLE:
            if self.input_cmd.linear.x != 0 or self.input_cmd.angular.z != 0:
                if distances['front'] < self.obstacle_distance:
                    self.change_state(BehaviorState.AVOIDING_OBSTACLE)
                else:
                    self.change_state(BehaviorState.MOVING)
        
        elif self.current_state == BehaviorState.MOVING:
            if distances['front'] < self.obstacle_distance:
                self.change_state(BehaviorState.AVOIDING_OBSTACLE)
            elif self.input_cmd.linear.x == 0 and self.input_cmd.angular.z == 0:
                self.change_state(BehaviorState.IDLE)
        
        elif self.current_state == BehaviorState.AVOIDING_OBSTACLE:
            if distances['front'] > self.obstacle_distance * 1.5:  # Histeresis
                self.change_state(BehaviorState.MOVING)
            elif (current_time - self.state_start_time).nanoseconds / 1e9 > 5.0:  # 5 saniye
                self.change_state(BehaviorState.TURNING)
        
        elif self.current_state == BehaviorState.TURNING:
            if (current_time - self.state_start_time).nanoseconds / 1e9 > self.turn_duration:
                self.change_state(BehaviorState.MOVING)
        
        elif self.current_state == BehaviorState.STUCK:
            if (current_time - self.state_start_time).nanoseconds / 1e9 > 3.0:  # 3 saniye
                self.change_state(BehaviorState.TURNING)
        
        elif self.current_state == BehaviorState.EMERGENCY:
            if not distances['emergency_zone']:
                self.change_state(BehaviorState.IDLE)

    def change_state(self, new_state: BehaviorState):
        """Durum değiştir"""
        if new_state != self.current_state:
            self.previous_state = self.current_state
            self.current_state = new_state
            self.state_start_time = self.get_clock().now()
            
            self.get_logger().info(f"Davranış durumu değişti: {self.previous_state.value} -> {new_state.value}")
            
            # Durum geçiş eylemleri
            if new_state == BehaviorState.EMERGENCY:
                self.publish_emergency(True)
            elif self.previous_state == BehaviorState.EMERGENCY:
                self.publish_emergency(False)

    def generate_behavior_command(self, distances: dict) -> Twist:
        """Davranışa göre komut oluştur"""
        cmd = Twist()
        
        if self.current_state == BehaviorState.IDLE:
            # Durgun kalıp bekle
            pass
        
        elif self.current_state == BehaviorState.MOVING:
            # Normal hareket - giriş komutunu geçir
            cmd = self.input_cmd
        
        elif self.current_state == BehaviorState.AVOIDING_OBSTACLE:
            # Engel kaçınma davranışı
            cmd = self.obstacle_avoidance_behavior(distances)
        
        elif self.current_state == BehaviorState.STUCK:
            # Geri git ve dön
            cmd.linear.x = -0.3  # Yavaşça geri git
            cmd.angular.z = 0.5  # Sola dön
        
        elif self.current_state == BehaviorState.TURNING:
            # Dönerek yeni yön bul
            if distances['left'] > distances['right']:
                cmd.angular.z = 0.8  # Sola dön
            else:
                cmd.angular.z = -0.8  # Sağa dön
        
        elif self.current_state == BehaviorState.EMERGENCY:
            # Acil durum - dur
            pass
        
        # Güvenlik limitleri uygula
        cmd.linear.x = max(-1.0, min(cmd.linear.x, 1.5))
        cmd.angular.z = max(-2.0, min(cmd.angular.z, 2.0))
        
        return cmd

    def obstacle_avoidance_behavior(self, distances: dict) -> Twist:
        """Gelişmiş engel kaçınma davranışı"""
        cmd = Twist()
        
        # Acil durum kontrolü
        if distances['emergency_zone'] or distances['front'] < self.emergency_distance:
            # Acil dur ve geri git
            cmd.linear.x = -0.3
            cmd.angular.z = 0.0
            self.get_logger().warn("ACİL DURUM: Çok yakın engel!")
            return cmd
        
        # Önde engel var mı?
        if distances['front'] < self.obstacle_distance:
            # Durma ve dönme stratejisi
            cmd.linear.x = 0.0  # Dur
            
            # En iyi dönüş yönünü belirle
            left_clear = distances['left'] > self.obstacle_distance * 1.5
            right_clear = distances['right'] > self.obstacle_distance * 1.5
            front_left_clear = distances['front_left'] > self.obstacle_distance
            front_right_clear = distances['front_right'] > self.obstacle_distance
            
            if left_clear and front_left_clear:
                # Sol taraf daha açık
                cmd.angular.z = 0.8
                self.get_logger().info("Sola dönüyor - sol taraf açık")
            elif right_clear and front_right_clear:
                # Sağ taraf daha açık
                cmd.angular.z = -0.8
                self.get_logger().info("Sağa dönüyor - sağ taraf açık")
            elif distances['left'] > distances['right']:
                # Sol taraf daha uzak
                cmd.angular.z = 0.6
                self.get_logger().info("Sola dönüyor - sol daha uzak")
            else:
                # Sağ taraf daha uzak
                cmd.angular.z = -0.6
                self.get_logger().info("Sağa dönüyor - sağ daha uzak")
                
        elif distances['front'] < self.obstacle_distance * 1.5:
            # Orta mesafede engel - yavaşla ve hafif dön
            cmd.linear.x = 0.3
            
            if distances['left'] > distances['right']:
                cmd.angular.z = 0.3
            else:
                cmd.angular.z = -0.3
                
        else:
            # Önde engel yok, normal ilerleme
            cmd.linear.x = 0.6
            
            # Duvar takibi optimizasyonu
            if distances['right'] < 1.2 and distances['right'] > 0.4:
                # Sağ duvar takibi
                cmd.angular.z = 0.1
            elif distances['left'] < 1.2 and distances['left'] > 0.4:
                # Sol duvar takibi
                cmd.angular.z = -0.1
            else:
                cmd.angular.z = 0.0
        
        # Güvenlik sınırları
        cmd.linear.x = max(-0.5, min(0.8, cmd.linear.x))
        cmd.angular.z = max(-1.2, min(1.2, cmd.angular.z))
        
        return cmd

    def is_stuck(self) -> bool:
        """Robot sıkışmış mı kontrol et"""
        # Hareket hızını kontrol et
        speed = abs(self.input_cmd.linear.x) + abs(self.input_cmd.angular.z)
        
        if speed < self.stuck_threshold:
            return False  # Zaten hareket etmiyor
        
        # Pozisyon geçmişini kontrol et (basit implementasyon)
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_movement_time).nanoseconds / 1e9
        
        if time_diff > self.stuck_time_threshold:
            return True
        
        # Eğer hızlı hareket ediyorsa, hareket zamanını güncelle
        if speed > self.stuck_threshold * 2:
            self.last_movement_time = current_time
        
        return False

    def publish_emergency(self, emergency: bool):
        """Acil durum durumunu yayınla"""
        msg = Bool()
        msg.data = emergency
        self.emergency_pub.publish(msg)
        
        if emergency:
            self.get_logger().warn("ACİL DURUM AKTİF!")
        else:
            self.get_logger().info("Acil durum sıfırlandı")

    def publish_status(self):
        """Davranış durumunu yayınla"""
        status = {
            "current_state": self.current_state.value,
            "previous_state": self.previous_state.value,
            "emergency": self.current_state == BehaviorState.EMERGENCY,
            "stuck": self.current_state == BehaviorState.STUCK,
            "state_duration": (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.behavior_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = BehaviorManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Davranış Yöneticisi durduruluyor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
