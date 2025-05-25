#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool
import sys
import threading
from enum import Enum


class ControlInterface(Node):
    def __init__(self):
        super().__init__('control_interface')
        
        # Publishers
        self.mode_pub = self.create_publisher(String, '/driving_mode', 10)
        self.manual_cmd_pub = self.create_publisher(Twist, '/manual_cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.emergency_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        
        # Subscribers
        self.status_sub = self.create_subscription(
            String,
            '/autonomous_status',
            self.status_callback,
            10
        )
        
        self.behavior_status_sub = self.create_subscription(
            String,
            '/behavior_status',
            self.behavior_status_callback,
            10
        )
        
        # Durum değişkenleri
        self.autonomous_status = {}
        self.behavior_status = {}
        self.current_mode = "manual"
        self.emergency_active = False
        
        # Kullanıcı arayüzü thread
        self.ui_thread = threading.Thread(target=self.user_interface_loop)
        self.ui_thread.daemon = True
        self.running = True
        
        self.get_logger().info("Kontrol Arayüzü başlatıldı")
        self.print_help()
        
        # Thread'i başlat
        self.ui_thread.start()

    def status_callback(self, msg):
        """Otonom durum callback"""
        try:
            import json
            self.autonomous_status = json.loads(msg.data)
        except:
            pass

    def behavior_status_callback(self, msg):
        """Davranış durum callback"""
        try:
            import json
            self.behavior_status = json.loads(msg.data)
        except:
            pass

    def print_help(self):
        """Yardım mesajını yazdır"""
        print("\n" + "="*60)
        print("🤖 TANK ROBOT KONTROL ARAYÜZÜ")
        print("="*60)
        print("SÜRÜŞ MODLARI:")
        print("  m  - Manuel mod")
        print("  a  - Otonom mod")
        print("  w  - Duvar takip modu")
        print("  e  - Keşif modu")
        print("")
        print("MANUEL KONTROL (Manuel modda):")
        print("  w  - İleri")
        print("  s  - Geri")
        print("  a  - Sola dön")
        print("  d  - Sağa dön")
        print("  x  - Dur")
        print("")
        print("OTONOM KONTROL:")
        print("  g  - Hedef belirle (x,y koordinatı)")
        print("")
        print("ACİL DURUM:")
        print("  SPACE - Acil durum toggle")
        print("")
        print("DİĞER:")
        print("  h  - Bu yardımı göster")
        print("  i  - Durum bilgisi")
        print("  q  - Çıkış")
        print("="*60)

    def user_interface_loop(self):
        """Kullanıcı arayüzü döngüsü"""
        while self.running:
            try:
                print(f"\n[{self.current_mode.upper()}] Komut: ", end="", flush=True)
                
                # Non-blocking input simulation
                import select
                import sys
                
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    command = sys.stdin.readline().strip().lower()
                    self.process_command(command)
                else:
                    continue
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f"UI hatası: {e}")

    def process_command(self, command):
        """Komut işleme"""
        if command == 'q':
            self.running = False
            self.get_logger().info("Çıkış yapılıyor...")
            rclpy.shutdown()
            return
        
        elif command == 'h':
            self.print_help()
        
        elif command == 'i':
            self.print_status()
        
        elif command == ' ':  # Space
            self.toggle_emergency()
        
        # Mod değiştirme komutları
        elif command == 'm':
            self.change_mode("manual")
        elif command == 'a':
            self.change_mode("autonomous")
        elif command == 'w' and self.current_mode != "manual":
            self.change_mode("follow_wall")
        elif command == 'e':
            self.change_mode("explore")
        
        # Manuel kontrol komutları
        elif self.current_mode == "manual":
            self.handle_manual_command(command)
        
        # Hedef belirleme
        elif command == 'g':
            self.set_goal()
        
        else:
            print(f"Bilinmeyen komut: {command}")

    def change_mode(self, mode):
        """Sürüş modunu değiştir"""
        self.current_mode = mode
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        print(f"Mod değiştirildi: {mode.upper()}")

    def handle_manual_command(self, command):
        """Manuel komutları işle"""
        cmd = Twist()
        
        if command == 'w':
            cmd.linear.x = 1.0
            print("İleri")
        elif command == 's':
            cmd.linear.x = -0.5
            print("Geri")
        elif command == 'a':
            cmd.angular.z = 1.0
            print("Sola dön")
        elif command == 'd':
            cmd.angular.z = -1.0
            print("Sağa dön")
        elif command == 'x':
            print("Dur")
        else:
            print(f"Manuel modda geçersiz komut: {command}")
            return
        
        self.manual_cmd_pub.publish(cmd)

    def set_goal(self):
        """Hedef koordinatı belirle"""
        try:
            coords = input("Hedef koordinatlarını girin (x,y): ").strip()
            x, y = map(float, coords.split(','))
            
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'
            goal_msg.pose.position.x = x
            goal_msg.pose.position.y = y
            goal_msg.pose.orientation.w = 1.0
            
            self.goal_pub.publish(goal_msg)
            print(f"Hedef belirlendi: ({x:.2f}, {y:.2f})")
            
        except ValueError:
            print("Geçersiz koordinat formatı! Örnek: 5.0,3.0")
        except Exception as e:
            print(f"Hedef belirleme hatası: {e}")

    def toggle_emergency(self):
        """Acil durumu toggle et"""
        self.emergency_active = not self.emergency_active
        msg = Bool()
        msg.data = self.emergency_active
        self.emergency_pub.publish(msg)
        
        if self.emergency_active:
            print("🚨 ACİL DURUM AKTİF!")
        else:
            print("✅ Acil durum sıfırlandı")

    def print_status(self):
        """Durum bilgisini yazdır"""
        print("\n" + "="*50)
        print("📊 ROBOT DURUM BİLGİSİ")
        print("="*50)
        print(f"Mevcut Mod: {self.current_mode.upper()}")
        print(f"Acil Durum: {'🚨 AKTİF' if self.emergency_active else '✅ Pasif'}")
        
        if self.autonomous_status:
            print("\n🚗 Otonom Durum:")
            print(f"  Pozisyon: ({self.autonomous_status.get('position', {}).get('x', 0):.2f}, "
                  f"{self.autonomous_status.get('position', {}).get('y', 0):.2f})")
            print(f"  Yön açısı: {self.autonomous_status.get('position', {}).get('yaw', 0):.2f} rad")
            print(f"  Yol durumu: {'✅ Açık' if self.autonomous_status.get('path_clear', False) else '❌ Engelli'}")
            print(f"  Keşif ilerlemesi: {self.autonomous_status.get('exploration_progress', '0/0')}")
        
        if self.behavior_status:
            print("\n🧠 Davranış Durumu:")
            print(f"  Mevcut davranış: {self.behavior_status.get('current_state', 'unknown').upper()}")
            print(f"  Önceki davranış: {self.behavior_status.get('previous_state', 'unknown').upper()}")
            print(f"  Davranış süresi: {self.behavior_status.get('state_duration', 0):.1f} saniye")
            print(f"  Sıkışma durumu: {'❌ Sıkışmış' if self.behavior_status.get('stuck', False) else '✅ Normal'}")
        
        print("="*50)


def main(args=None):
    rclpy.init(args=args)
    
    node = ControlInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
