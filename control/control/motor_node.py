#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import socket
import json
import threading

"""
# İleri
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Sola dönüş  
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'

# Dur
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
"""
class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        
        # Motor kontrol parametreleri
        self.max_speed = 100.0  # Maksimum motor hızı
        self.wheel_base = 0.5   # Tekerlekler arası mesafe (metre)
        
        # Unity TCP bağlantısı
        self.unity_socket = None
        self.unity_host = '127.0.0.1'  # localhost
        self.unity_port = 9999
        self.connection_retry_interval = 5.0  # 5 saniyede bir yeniden dene
        self.last_connection_attempt = 0.0
        self.is_connected = False
        self.setup_unity_connection()
        
        # Subscriptions
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publishers
        self.motor_speeds_pub = self.create_publisher(
            Float32MultiArray,
            '/motor_speeds',
            10
        )
        
        # Timer for regular motor updates
        self.timer = self.create_timer(0.1, self.update_motors)  # 10 Hz
        
        # Timer for connection monitoring (sadece bağlantı yoksa çalışır)
        self.connection_timer = self.create_timer(2.0, self.check_connection)  # 2 saniyede bir
        
        # Motor hızları
        self.left_speed = 0.0
        self.right_speed = 0.0
        
        self.get_logger().info("Motor Control Node başlatıldı (Unity simülasyon modu)")

    def setup_unity_connection(self):
        """Unity ile TCP bağlantısı kur"""
        try:
            if self.unity_socket:
                try:
                    self.unity_socket.close()
                except:
                    pass
            
            self.unity_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.unity_socket.settimeout(5.0)  # 5 saniye timeout
            self.unity_socket.connect((self.unity_host, self.unity_port))
            self.is_connected = True
            self.get_logger().info(f"Unity bağlantısı kuruldu: {self.unity_host}:{self.unity_port}")
        except Exception as e:
            self.get_logger().warn(f"Unity bağlantısı kurulamadı: {e}")
            self.unity_socket = None
            self.is_connected = False
            self.last_connection_attempt = self.get_clock().now().nanoseconds / 1e9

    def check_connection(self):
        """Bağlantı durumunu kontrol et ve gerekirse yeniden bağlan"""
        if not self.is_connected or self.unity_socket is None:
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # Son deneme zamanından belli bir süre geçtiyse tekrar dene
            if current_time - self.last_connection_attempt > self.connection_retry_interval:
                self.get_logger().info("Unity bağlantısı yeniden kuruluyor...")
                self.setup_unity_connection()
        
        # Bağlantı testini yap
        elif self.unity_socket:
            try:
                # Basit bir test mesajı gönder
                test_data = {
                    "type": "connection_test",
                    "timestamp": self.get_clock().now().nanoseconds / 1e9
                }
                test_command = json.dumps(test_data) + '\n'
                self.unity_socket.send(test_command.encode())
            except Exception as e:
                self.get_logger().warn(f"Bağlantı testi başarısız, yeniden bağlanılıyor: {e}")
                self.is_connected = False
                self.unity_socket = None

    def send_to_unity(self, data):
        """Unity'ye güvenli veri gönderimi"""
        if not self.is_connected or self.unity_socket is None:
            return False
        
        try:
            command = json.dumps(data) + '\n'
            self.unity_socket.send(command.encode())
            return True
        except Exception as e:
            self.get_logger().error(f"Unity gönderim hatası: {e}")
            self.is_connected = False
            self.unity_socket = None
            return False

    def cmd_vel_callback(self, msg):
        """Twist mesajından motor hızlarını hesapla"""
        linear_x = msg.linear.x    # İleri/geri hız
        angular_z = msg.angular.z  # Dönüş hızı
        
        # Tank drive kinematik hesaplama
        left_vel = linear_x - (angular_z * self.wheel_base / 2.0)
        right_vel = linear_x + (angular_z * self.wheel_base / 2.0)
        
        # Hızları normalize et (-100 ile +100 arası)
        self.left_speed = max(-self.max_speed, min(self.max_speed, left_vel * 100))
        self.right_speed = max(-self.max_speed, min(self.max_speed, right_vel * 100))
        
        self.get_logger().info(f"Motor hızları - Sol: {self.left_speed:.2f}, Sağ: {self.right_speed:.2f}")
        self.get_logger().info(f"Gelen komut - Linear: {linear_x:.2f}, Angular: {angular_z:.2f}")

    def update_motors(self):
        """Motor hızlarını güncelle ve gönder"""
        # Motor hızlarını publish et
        motor_msg = Float32MultiArray()
        motor_msg.data = [self.left_speed, self.right_speed]
        self.motor_speeds_pub.publish(motor_msg)
        
        # Unity'ye TCP üzerinden gönder
        motor_data = {
            "type": "motor_control",
            "left_speed": self.left_speed,
            "right_speed": self.right_speed
        }
        
        success = self.send_to_unity(motor_data)
        if not success and not self.is_connected:
            # Bağlantı kopmuşsa logla ama her seferinde spam yapma
            pass  # check_connection timer'ı zaten yeniden bağlanmaya çalışacak

    def manual_control(self, left_power, right_power):
        """Manuel motor kontrolü"""
        self.left_speed = max(-self.max_speed, min(self.max_speed, left_power))
        self.right_speed = max(-self.max_speed, min(self.max_speed, right_power))

    def stop_motors(self):
        """Motorları durdur"""
        self.left_speed = 0.0
        self.right_speed = 0.0

    def destroy_node(self):
        """Node kapatılırken motorları durdur"""
        self.stop_motors()
        if self.unity_socket and self.is_connected:
            try:
                # Son durma komutu gönder
                stop_data = {
                    "type": "motor_control",
                    "left_speed": 0.0,
                    "right_speed": 0.0
                }
                self.send_to_unity(stop_data)
                self.unity_socket.close()
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    motor_node = MotorControlNode()
    
    try:
        rclpy.spin(motor_node)
    except KeyboardInterrupt:
        motor_node.get_logger().info("Motor Control Node durduruluyor...")
    finally:
        motor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()