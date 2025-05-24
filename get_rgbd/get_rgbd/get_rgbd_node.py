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
PORT = 5005
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
        
        # RGB-D verileri için değişkenler
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
                conn, addr = server.accept()
                self.get_logger().info(f"RGB-D TCP connection from {addr}")
                try:
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
                        
                except Exception as e:
                    self.get_logger().error(f"RGB-D TCP error: {e}")
                finally:
                    conn.close()
        except Exception as e:
            self.get_logger().error(f"RGB-D TCP server başlatılamadı: {e}")
            time.sleep(5)

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
                        
                        # Corrected Unity -> ROS dönüşümü
                        # Unity (right-handed, Y-up) to ROS (right-handed, Z-up)
                        ros_x = unity_z  # Unity forward → ROS forward
                        ros_y = -unity_x  # Unity right → ROS left
                        ros_z = unity_y  # Unity up → ROS up
                        
                        points.append((ros_x, ros_y, ros_z, r, g, b))
            
            with self.lock:
                self.points = points
            
        except Exception as e:
            self.get_logger().error(f"RGB-D veri işleme hatası: {e}")
            self.get_logger().error(f"İlk birkaç satır: {lines[:10] if len(lines) >= 10 else lines}")

    def timer_callback(self):
        with self.lock:
            if not self.points:
                return

            # Header
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.frame_id

            # PointCloud2 mesajı için nokta bulutu verileri hazırla
            self.publish_point_cloud(header)

            # RGB ve Derinlik görüntüleri oluştur ve yayınla
            self.publish_rgb_depth_images(header)

            # CameraInfo mesajı oluştur ve yayınla
            self.publish_camera_info(header)

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
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Her nokta için binary veri oluştur
        data = bytearray(point_count * 16)  # 16 bytes per point (x,y,z,rgb)
        
        for i, point in enumerate(self.points):
            x, y, z, r, g, b = point
            
            # x, y, z koordinatları
            struct.pack_into('fff', data, i * 16, x, y, z)
            
            # RGB renk değerleri (bitfield olarak paketlenmiş)
            r_int = int(r * 255.0)
            g_int = int(g * 255.0)
            b_int = int(b * 255.0)
            rgb = (r_int << 16) | (g_int << 8) | b_int
            struct.pack_into('f', data, i * 16 + 12, rgb)
        
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
        depth_data = np.zeros((self.height, self.width), dtype=np.float32)
        depth_mask = np.zeros((self.height, self.width), dtype=bool)
        
        # Noktaları bir 2D ızgara gibi organize et
        h_angle_range = self.horizontal_fov
        v_angle_range = self.vertical_fov
        
        for point in self.points:
            x, y, z, r, g, b = point
            
            # Açıları hesapla (halen ROS koordinat sisteminde)
            # x ekseni ileriyi, y ekseni solu, z ekseni yukarıyı gösterir
            distance = np.sqrt(x*x + y*y + z*z)
            if distance < 0.01:  # Çok yakın noktaları atla
                continue
                
            # Yatay açı (yaw) - y eksenindeki sapma
            h_angle = np.arctan2(y, x)
            # Dikey açı (pitch) - z eksenindeki yükselme
            v_angle = np.arcsin(z / distance)
            
            # Açıları görüntü koordinatlarına dönüştür - yatay açı sağdan sola
            h_idx = int((h_angle / np.radians(h_angle_range) + 0.5) * self.width)
            
            # Dikey açı - ÖNEMLİ DEĞİŞİKLİK: Dikey yansımayı düzeltmek için
            # Dikey açı dönüşümünü tersine çevir: Artı açılar (yukarı) görüntünün üstüne gitsin
            v_idx = int((-v_angle / np.radians(v_angle_range) + 0.5) * self.height)
            
            # Görüntü sınırlarını kontrol et
            if 0 <= h_idx < self.width and 0 <= v_idx < self.height:
                if not depth_mask[v_idx, h_idx] or depth_data[v_idx, h_idx] > distance:
                    # BGR sıralaması ile doldur
                    rgb_data[v_idx, h_idx, 0] = int(b * 255)  # B
                    rgb_data[v_idx, h_idx, 1] = int(g * 255)  # G
                    rgb_data[v_idx, h_idx, 2] = int(r * 255)  # R
                    depth_data[v_idx, h_idx] = distance
                    depth_mask[v_idx, h_idx] = True
        
        # RGB görüntü mesajı
        rgb_msg = Image()
        rgb_msg.header = header
        rgb_msg.height = self.height
        rgb_msg.width = self.width
        rgb_msg.encoding = 'bgr8'  # <-- Burayı değiştirin!
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

        # Focal length and principal point estimation
        # If you have real intrinsics, use them here!
        fx = fy = 0.5 * self.width / np.tan(np.radians(self.horizontal_fov) / 2) if self.horizontal_fov > 0 else 525.0
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
