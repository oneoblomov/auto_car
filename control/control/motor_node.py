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
        
        # Motor hızları
        self.left_speed = 0.0
        self.right_speed = 0.0
        
        self.get_logger().info("Motor Control Node başlatıldı (Unity simülasyon modu)")

    def setup_unity_connection(self):
        """Unity ile TCP bağlantısı kur"""
        try:
            self.unity_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.unity_socket.connect((self.unity_host, self.unity_port))
            self.get_logger().info(f"Unity bağlantısı kuruldu: {self.unity_host}:{self.unity_port}")
        except Exception as e:
            self.get_logger().warn(f"Unity bağlantısı kurulamadı: {e}")
            self.unity_socket = None

    def cmd_vel_callback(self, msg):
        """Twist mesajından motor hızlarını hesapla"""
        linear_x = msg.linear.x    # İleri/geri hız
        angular_z = msg.angular.z  # Dönüş hızı
        
        # Tank drive kinematik hesaplama
        left_vel = linear_x - (angular_z * self.wheel_base / 2.0)
        right_vel = linear_x + (angular_z * self.wheel_base / 2.0)
        
        # Hızları normalize et
        self.left_speed = max(-self.max_speed, min(self.max_speed, left_vel * 100))
        self.right_speed = max(-self.max_speed, min(self.max_speed, right_vel * 100))
        
        self.get_logger().debug(f"Motor hızları - Sol: {self.left_speed:.2f}, Sağ: {self.right_speed:.2f}")

    def update_motors(self):
        """Motor hızlarını güncelle ve gönder"""
        # Motor hızlarını publish et
        motor_msg = Float32MultiArray()
        motor_msg.data = [self.left_speed, self.right_speed]
        self.motor_speeds_pub.publish(motor_msg)
        
        # Unity'ye TCP üzerinden gönder
        if self.unity_socket:
            try:
                motor_data = {
                    "type": "motor_control",
                    "left_speed": self.left_speed,
                    "right_speed": self.right_speed
                }
                command = json.dumps(motor_data) + '\n'
                self.unity_socket.send(command.encode())
            except Exception as e:
                self.get_logger().error(f"Unity gönderim hatası: {e}")
                self.unity_socket = None
                # Yeniden bağlanmaya çalış
                self.setup_unity_connection()

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
        if self.unity_socket:
            try:
                # Son durma komutu gönder
                stop_data = {
                    "type": "motor_control",
                    "left_speed": 0.0,
                    "right_speed": 0.0
                }
                self.unity_socket.send((json.dumps(stop_data) + '\n').encode())
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