#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs.msg import CameraInfo  # <-- Add this import
from std_msgs.msg import Header
import socket
import threading
import struct
import numpy as np
import time

HOST = '127.0.0.1'
PORT = 5005  # Unity'deki port ile eşleştir
class RGBDNode(Node):
    def __init__(self):
        super().__init__('get_rgbd')
        
        # Yayıncılar
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2, '/get_rgbd/points', 10)
        self.rgb_image_publisher = self.create_publisher(
            Image, '/get_rgbd/rgb', 10)
        self.depth_image_publisher = self.create_publisher(
            Image, '/get_rgbd/depth', 10)
        # CameraInfo publisher
        self.camera_info_publisher = self.create_publisher(
            CameraInfo, '/get_rgbd/camera_info', 10)
        
        self.points = []  # Nokta bulutu verileri (x, y, z, r, g, b)
        self.width = 0
        self.height = 0
        self.frame_id = 'camera_link'
        self.horizontal_fov = 0.0
        self.vertical_fov = 0.0
        self.angle_step = 0.0
        self.max_range = 50.0
        
        # Thread-safe erişim için kilit
        self.lock = threading.Lock()
        
        # TCP soket serveri için thread
        self.tcp_thread = threading.Thread(target=self.tcp_server, daemon=True)
        self.tcp_thread.start()
        
        # Periyodik yayın için timer
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
        # Bağlantı sayacı
        self.connection_count = 0
        
        self.get_logger().info("RGB-D node başlatıldı, bağlantı için hazır.")

    def tcp_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            server.bind((HOST, PORT))
            server.listen(1)
            self.get_logger().info(f"RGB-D TCP server listening on {HOST}:{PORT}")
            
            while rclpy.ok():
                conn = None
                try:
                    conn, addr = server.accept()
                    self.get_logger().info(f"RGB-D TCP connection from {addr}")
                    
                    # Uzunluk değerini al
                    length_bytes = conn.recv(4)
                    if not length_bytes or len(length_bytes) < 4:
                        self.get_logger().warning(f"Eksik uzunluk verisi: {len(length_bytes) if length_bytes else 0} bytes alındı")
                        conn.close()
                        continue
                    
                    message_length = struct.unpack("i", length_bytes)[0]
                    
                    if message_length <= 0 or message_length > 10000000:  # Makul bir üst sınır (10MB)
                        self.get_logger().warning(f"Geçersiz mesaj uzunluğu: {message_length}")
                        conn.close()
                        continue
                        
                    self.get_logger().info(f"Beklenen veri uzunluğu: {message_length} bytes")
                    
                    # Veriyi al
                    data = b''
                    bytes_received = 0
                    
                    while bytes_received < message_length:
                        chunk = conn.recv(min(4096, message_length - bytes_received))
                        if not chunk:
                            break
                        data += chunk
                        bytes_received += len(chunk)
                        
                    if bytes_received == message_length:
                        data_str = data.decode('utf-8')
                        self.parse_rgbd_data(data_str)
                        self.connection_count += 1
                        self.get_logger().info(f"RGB-D data received (#{self.connection_count}): {bytes_received} bytes")
                    else:
                        self.get_logger().warning(f"Eksik veri: {bytes_received}/{message_length} bytes alındı")
                        
                except socket.timeout:
                    continue
                except Exception as e:
                    self.get_logger().error(f"Connection error: {e}")
                finally:
                    try:
                        if conn:
                            conn.close()
                    except:
                        pass
                        pass
        except Exception as e:
            self.get_logger().error(f"RGB-D TCP server başlatılamadı: {e}")
        finally:
            try:
                server.close()
            except:
                pass

    def parse_rgbd_data(self, data_str):
        lines = data_str.strip().split('\n')  # Changed from '\\n' to '\n'
        
        try:
            line_idx = 0
            
            # Frame ID
            self.frame_id = lines[line_idx]
            line_idx += 1
            
            # Timestamp
            timestamp = float(lines[line_idx])
            line_idx += 1
            
            # Görüntü boyutları
            self.width = int(lines[line_idx])
            line_idx += 1
            self.height = int(lines[line_idx])
            line_idx += 1
            
            # Tarama ayarları
            self.horizontal_fov = float(lines[line_idx])
            line_idx += 1
            self.vertical_fov = float(lines[line_idx])
            line_idx += 1
            self.angle_step = float(lines[line_idx])
            line_idx += 1
            self.max_range = float(lines[line_idx])
            line_idx += 1
            
            # Nokta sayısı
            point_count = int(lines[line_idx])
            line_idx += 1
            
            # Noktaları oku
            points = []
            for i in range(point_count):
                if line_idx + i < len(lines) and lines[line_idx + i]:
                    point_data = lines[line_idx + i].split(',')
                    if len(point_data) >= 6:
                        unity_x = float(point_data[0])
                        unity_y = float(point_data[1])
                        unity_z = float(point_data[2])
                        r = float(point_data[3])
                        g = float(point_data[4])
                        b = float(point_data[5])
                        
                        # Unity -> ROS koordinat dönüşümü
                        # Unity (left-handed, Y-up): +X=sağ, +Y=yukarı, +Z=ileri
                        # ROS (right-handed, Z-up): +X=ileri, +Y=sol, +Z=yukarı
                        ros_x = unity_z   # Unity Z (ileri) -> ROS X (ileri)  
                        ros_y = -unity_x  # Unity X (sağ) -> ROS -Y (sol)
                        ros_z = unity_y   # Unity Y (yukarı) -> ROS Z (yukarı)
                        
                        points.append((ros_x, ros_y, ros_z, r, g, b))
            
            with self.lock:
                self.points = points
            
        except Exception as e:
            self.get_logger().error(f"RGB-D veri işleme hatası: {e}")
            self.get_logger().error(f"İlk birkaç satır: {lines[:10] if len(lines) >= 10 else lines}")

    def timer_callback(self):
        with self.lock:
            if not self.points:
                # Veri yoksa boş mesajlar yayınla (debug için)
                if self.connection_count == 0:
                    self.get_logger().debug("RGB-D verisi bekleniyor...")
                return

            # Header
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.frame_id

            try:
                # PointCloud2 mesajı için nokta bulutu verileri hazırla
                self.publish_point_cloud(header)

                # RGB ve Derinlik görüntüleri oluştur ve yayınla
                self.publish_rgb_depth_images(header)

                # CameraInfo mesajı oluştur ve yayınla
                self.publish_camera_info(header)
                
            except Exception as e:
                self.get_logger().error(f"Timer callback hatası: {e}")

    def publish_point_cloud(self, header):
        if not self.points:
            return
        
        # PointCloud2 mesajı
        pc2 = PointCloud2()
        pc2.header = header
        
        # Nokta sayısı ve boyutlar
        point_count = len(self.points)
        
        # PointCloud2 alanlar (x,y,z,rgb)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),  # UINT32 kullan
        ]
        
        # Her nokta için binary veri oluştur
        data = bytearray(point_count * 16)  # 16 bytes per point (x,y,z,rgb)
        
        for i, point in enumerate(self.points):
            x, y, z, r, g, b = point
            
            # x, y, z koordinatları
            struct.pack_into('fff', data, i * 16, x, y, z)
            
            # RGB renk değerleri (bitfield olarak paketlenmiş)
            r_int = int(np.clip(r * 255.0, 0, 255))
            g_int = int(np.clip(g * 255.0, 0, 255)) 
            b_int = int(np.clip(b * 255.0, 0, 255))
            rgb = (r_int << 16) | (g_int << 8) | b_int
            struct.pack_into('I', data, i * 16 + 12, rgb)  # Unsigned int kullan
        
        # PointCloud2 mesajını doldur
        pc2.height = 1
        pc2.width = point_count
        pc2.is_dense = False
        pc2.is_bigendian = False
        pc2.fields = fields
        pc2.point_step = 16
        pc2.row_step = pc2.point_step * pc2.width
        pc2.data = bytes(data)
        
        # Yayınla
        self.pointcloud_publisher.publish(pc2)
        self.get_logger().debug(f"PointCloud2 yayınlandı: {point_count} nokta")
    
    def publish_rgb_depth_images(self, header):
        # Boyutları kontrol et
        if self.width <= 0 or self.height <= 0 or not self.points:
            return
        
        # RGB ve derinlik görüntüleri için np dizileri
        rgb_data = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        depth_data = np.full((self.height, self.width), np.inf, dtype=np.float32)  # Başlangıçta sonsuz mesafe
        
        # Açı aralıkları (radyan cinsinden)
        h_angle_range = np.radians(self.horizontal_fov)
        v_angle_range = np.radians(self.vertical_fov)
        
        for point in self.points:
            x, y, z, r, g, b = point
            
            # ROS koordinat sisteminde mesafe hesapla
            # x: ileri, y: sol, z: yukarı
            distance = np.sqrt(x*x + y*y + z*z)
            if distance < 0.01:  # Çok yakın noktaları atla
                continue
            
            # Kamera koordinat sisteminde açı hesaplama
            # Yatay açı: y/x oranından (sol-sağ)
            h_angle = np.arctan2(y, x)
            # Dikey açı: z bileşeninden (yukarı-aşağı) 
            v_angle = np.arctan2(z, np.sqrt(x*x + y*y))
            
            # Açı sınırlarını kontrol et
            if abs(h_angle) > h_angle_range/2 or abs(v_angle) > v_angle_range/2:
                continue
                
            # Açıları görüntü koordinatlarına dönüştür
            # Yatay: [-FOV/2, FOV/2] → [0, width-1]
            h_normalized = (h_angle + h_angle_range/2) / h_angle_range
            h_idx = int(np.clip(h_normalized * self.width, 0, self.width-1))
            
            # Dikey: [FOV/2, -FOV/2] → [0, height-1] (kamera görüntüsü ters)
            v_normalized = (-v_angle + v_angle_range/2) / v_angle_range
            v_idx = int(np.clip(v_normalized * self.height, 0, self.height-1))
            
            # En yakın noktayı kullan (Z-buffer)
            if depth_data[v_idx, h_idx] > distance:
                # BGR sıralaması ile doldur (OpenCV formatı)
                rgb_data[v_idx, h_idx, 0] = int(np.clip(b * 255, 0, 255))  # B
                rgb_data[v_idx, h_idx, 1] = int(np.clip(g * 255, 0, 255))  # G
                rgb_data[v_idx, h_idx, 2] = int(np.clip(r * 255, 0, 255))  # R
                depth_data[v_idx, h_idx] = distance
        
        # Sonsuz mesafeleri 0 yap (geçersiz piksel)
        depth_data[depth_data == np.inf] = 0.0
        
        # RGB görüntü mesajı
        rgb_msg = Image()
        rgb_msg.header = header
        rgb_msg.height = self.height
        rgb_msg.width = self.width
        rgb_msg.encoding = 'bgr8'
        rgb_msg.is_bigendian = 0
        rgb_msg.step = self.width * 3
        rgb_msg.data = rgb_data.tobytes()
        
        # Derinlik görüntü mesajı
        depth_msg = Image()
        depth_msg.header = header
        depth_msg.height = self.height
        depth_msg.width = self.width
        depth_msg.encoding = '32FC1'
        depth_msg.is_bigendian = 0
        depth_msg.step = self.width * 4
        depth_msg.data = depth_data.tobytes()
        
        # Yayınla
        self.rgb_image_publisher.publish(rgb_msg)
        self.depth_image_publisher.publish(depth_msg)
        self.get_logger().debug(f"RGB ve Depth görüntüleri yayınlandı: {self.width}x{self.height}, noktalar: {len(self.points)}")
        
        # İstatistikler ve debug bilgileri
        valid_pixels = np.sum(depth_data > 0)
        if valid_pixels > 0:
            min_depth = np.min(depth_data[depth_data > 0])
            max_depth = np.max(depth_data[depth_data > 0])
            avg_depth = np.mean(depth_data[depth_data > 0])
            
            self.get_logger().info(f"Derinlik haritası - Geçerli piksel: {valid_pixels}/{self.width*self.height}, "
                                   f"Min: {min_depth:.2f}m, Max: {max_depth:.2f}m, Ortalama: {avg_depth:.2f}m")
            
            # İlk birkaç geçerli noktanın detaylarını göster
            sample_count = 0
            for point in self.points[:5]:  # İlk 5 nokta
                x, y, z, r, g, b = point
                distance = np.sqrt(x*x + y*y + z*z)
                h_angle = np.arctan2(y, x)
                v_angle = np.arctan2(z, np.sqrt(x*x + y*y))
                
                self.get_logger().debug(f"Nokta {sample_count}: xyz=({x:.2f},{y:.2f},{z:.2f}), "
                                       f"mesafe={distance:.2f}m, açılar=({np.degrees(h_angle):.1f}°,{np.degrees(v_angle):.1f}°)")
                sample_count += 1
    def publish_camera_info(self, header):
        # Only publish if width/height are valid
        if self.width <= 0 or self.height <= 0:
            return

        cam_info = CameraInfo()
        cam_info.header = header
        cam_info.width = self.width
        cam_info.height = self.height
        cam_info.distortion_model = 'plumb_bob'
        cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion

        # Intrinsic parameters for perspective projection
        # FOV'dan focal length hesaplama (daha doğru)
        if self.horizontal_fov > 0 and self.vertical_fov > 0:
            fx = (self.width / 2.0) / np.tan(np.radians(self.horizontal_fov) / 2.0)
            fy = (self.height / 2.0) / np.tan(np.radians(self.vertical_fov) / 2.0)
        else:
            # Fallback values
            fx = fy = 525.0
            
        cx = self.width / 2.0
        cy = self.height / 2.0

        cam_info.k = [fx, 0, cx,
                      0, fy, cy,
                      0, 0, 1]
        cam_info.r = [1, 0, 0,
                      0, 1, 0,
                      0, 0, 1]
        cam_info.p = [fx, 0, cx, 0,
                      0, fy, cy, 0,
                      0, 0, 1, 0]

        self.camera_info_publisher.publish(cam_info)

def main(args=None):
    rclpy.init(args=args)
    node = RGBDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
