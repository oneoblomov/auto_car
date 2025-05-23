import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import socket
import threading
import struct
import time
import subprocess

class LidarNode(Node):
    def __init__(self):
        super().__init__('get_lidar')
        self.publisher_ = self.create_publisher(LaserScan, '/get_lidar/scan', 10)
        self.latest_ranges = []
        self.latest_frame_id = 'lidar_link'
        self.latest_angle_min = -3.14
        self.latest_angle_max = 3.14
        self.latest_angle_increment = 0.01
        self.latest_scan_time = 0.1
        self.latest_range_min = 0.1
        self.latest_range_max = 50.0
        self.lock = threading.Lock()
        # Değiştirilen soket yapılandırması
        self.tcp_thread = threading.Thread(target=self.tcp_server, daemon=True)
        self.tcp_thread.start()
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.connection_count = 0
        self.get_logger().info("LiDAR node başlatıldı, bağlantı için hazır.")

    def tcp_server(self):
        # IMU node'una benzer şekilde basitleştirilmiş sunucu kodu
        HOST = '127.0.0.1'  # IMU ile aynı yaklaşım - local host
        PORT = 5010
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            server.bind((HOST, PORT))
            server.listen(1)  # IMU gibi basit dinleme
            self.get_logger().info(f"LiDAR TCP server listening on {HOST}:{PORT}")
            
            while rclpy.ok():
                conn, addr = server.accept()
                self.get_logger().info(f"LiDAR TCP connection from {addr}")
                try:
                    # Uzunluk değerini al
                    length_bytes = conn.recv(4)
                    if not length_bytes:
                        self.get_logger().warning("Uzunluk verisi alınamadı")
                        conn.close()
                        continue
                    
                    message_length = struct.unpack("i", length_bytes)[0]
                    
                    # Veriyi al
                    data = b''
                    while len(data) < message_length:
                        chunk = conn.recv(min(1024, message_length - len(data)))
                        if not chunk:
                            break
                        data += chunk
                    
                    if len(data) == message_length:
                        data_str = data.decode('utf-8')
                        ranges = self.parse_unity_lidar_data(data_str)
                        if ranges:
                            with self.lock:
                                self.latest_ranges = ranges
                                self.get_logger().info(f"LiDAR veri alındı: {len(ranges)} nokta")
                    else:
                        self.get_logger().warning(f"Eksik veri: {len(data)}/{message_length} bytes alındı")
                        
                except Exception as e:
                    self.get_logger().error(f"LiDAR TCP error: {e}")
                finally:
                    conn.close()
        except Exception as e:
            self.get_logger().error(f"LiDAR TCP server başlatılamadı: {e}")
            time.sleep(5)

# recvall metodu kaldırıldı çünkü artık kullanılmıyor

    def parse_unity_lidar_data(self, data_str):
        lines = data_str.strip().split('\n')
        if len(lines) < 9:
            self.get_logger().warn(f"Not enough lines in LiDAR data: {len(lines)}")
            return []
            
        try:
            line_index = 0
            self.latest_frame_id = lines[line_index]
            line_index += 1
            
            timestamp = float(lines[line_index])  # Unity timestamp
            line_index += 1
            
            self.latest_angle_min = float(lines[line_index])
            line_index += 1
            self.latest_angle_max = float(lines[line_index])
            line_index += 1
            self.latest_angle_increment = float(lines[line_index])
            line_index += 1
            
            self.latest_scan_time = float(lines[line_index])
            line_index += 1
            time_increment = float(lines[line_index])  # time_increment değerini al
            line_index += 1
            
            self.latest_range_min = float(lines[line_index])
            line_index += 1
            self.latest_range_max = float(lines[line_index])
            line_index += 1
            
            num_ranges = int(lines[line_index])
            line_index += 1
            
            ranges = []
            for i in range(line_index, line_index + num_ranges):
                if i < len(lines):
                    try:
                        range_val = float(lines[i])
                        # Sonsuz değerleri maksimum menzile ayarla
                        if np.isinf(range_val):
                            range_val = self.latest_range_max
                        ranges.append(range_val)
                    except ValueError as e:
                        self.get_logger().error(f"Range değeri dönüştürülemedi: {lines[i]}, hata: {e}")
            
            # Topic ismini kontrol et (eğer varsa)
            if line_index + num_ranges < len(lines):
                topic_name = lines[line_index + num_ranges + 1]
                self.get_logger().debug(f"Topic name from Unity: {topic_name}")
            
            self.get_logger().debug(f"Parsed {len(ranges)} LiDAR points from Unity")
            return ranges
        except Exception as e:
            self.get_logger().error(f"Error parsing LiDAR data: {e}")
            if len(lines) > 0:
                self.get_logger().error(f"First few lines: {lines[:min(5, len(lines))]}")
            return []

    def parse_csv_data(self, csv):
        ranges = []
        lines = csv.strip().split('\n')
        for line in lines:
            if not line:
                continue
            parts = line.split(',')
            if len(parts) < 3:
                continue
            try:
                x = float(parts[0])
                y = float(parts[1])
                z = float(parts[2])
                if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(z)):
                    continue
                dist = (x**2 + y**2 + z**2) ** 0.5
                ranges.append(dist)
            except Exception:
                continue
        return ranges

    def timer_callback(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        with self.lock:
            msg.header.frame_id = self.latest_frame_id
            msg.angle_min = self.latest_angle_min
            msg.angle_max = self.latest_angle_max
            msg.angle_increment = self.latest_angle_increment
            msg.time_increment = 0.0
            msg.scan_time = self.latest_scan_time
            msg.range_min = self.latest_range_min
            msg.range_max = self.latest_range_max
            
            if self.latest_ranges:
                msg.ranges = self.latest_ranges.copy()
                msg.intensities = [0.0 for _ in msg.ranges]
                self.get_logger().debug(f"Publishing {len(msg.ranges)} LiDAR points")
            else:
                num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment)
                msg.ranges = [1.0 for _ in range(num_readings)]
                msg.intensities = [0.0 for _ in range(num_readings)]
                
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
