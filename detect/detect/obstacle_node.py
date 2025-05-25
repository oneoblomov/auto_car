#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Pose
import numpy as np
import cv2
from cv_bridge import CvBridge
import math


class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__('detect_obstacle')
        
        # Publishers
        self.obstacle_map_pub = self.create_publisher(
            OccupancyGrid,
            '/detect_obstacle/obstacle_map',
            10
        )
        
        # Subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/get_lidar/scan',
            self.lidar_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/get_rgbd_camera/depth_image',
            self.depth_callback,
            10
        )
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # Harita parametreleri
        self.map_width = 200  # piksel
        self.map_height = 200  # piksel
        self.map_resolution = 0.1  # metre/piksel
        self.robot_x = self.map_width // 2  # Robot merkez konumu
        self.robot_y = self.map_height // 2
        
        # Timer
        self.timer = self.create_timer(0.2, self.publish_obstacle_map)  # 5 Hz
        
        # Veri depolama
        self.latest_lidar = None
        self.latest_depth = None
        self.obstacle_map = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        
        self.get_logger().info("Engel Tespiti Node başlatıldı")

    def lidar_callback(self, msg):
        """LiDAR verisi ile engel tespiti"""
        self.latest_lidar = msg
        self.process_lidar_obstacles(msg)

    def depth_callback(self, msg):
        """Derinlik kamerası ile engel tespiti"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            self.get_logger().error(f"Depth image dönüştürme hatası: {e}")

    def process_lidar_obstacles(self, lidar_msg):
        """LiDAR verisi ile engel haritası oluştur"""
        # Haritayı temizle
        self.obstacle_map.fill(0)  # 0 = bilinmeyen
        
        angle = lidar_msg.angle_min
        
        for i, range_val in enumerate(lidar_msg.ranges):
            if range_val < lidar_msg.range_min or range_val > lidar_msg.range_max:
                angle += lidar_msg.angle_increment
                continue
            
            # Polar koordinatları kartezyen koordinatlara dönüştür
            x = range_val * math.cos(angle)
            y = range_val * math.sin(angle)
            
            # Harita koordinatlarına dönüştür
            map_x = int(self.robot_x + x / self.map_resolution)
            map_y = int(self.robot_y + y / self.map_resolution)
            
            # Harita sınırları içinde mi kontrol et
            if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                # Engel olarak işaretle
                if range_val < 3.0:  # 3 metre içindeki engeller
                    self.obstacle_map[map_y, map_x] = 100  # 100 = engel
                
                # Robot ile engel arasındaki alanı boş olarak işaretle
                self.mark_free_space(self.robot_x, self.robot_y, map_x, map_y)
            
            angle += lidar_msg.angle_increment

    def mark_free_space(self, x0, y0, x1, y1):
        """İki nokta arasındaki alanı boş olarak işaretle (Bresenham çizgi algoritması)"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                if self.obstacle_map[y, x] != 100:  # Engel değilse
                    self.obstacle_map[y, x] = -1  # -1 = boş alan
            
            if x == x1 and y == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def process_depth_obstacles(self):
        """Derinlik kamerası ile ek engel tespiti"""
        if self.latest_depth is None:
            return
            
        # Derinlik görüntüsünden engel tespiti
        height, width = self.latest_depth.shape
        
        # Alt yarı kısmı kontrol et (yer seviyesi)
        roi = self.latest_depth[height//2:, :]
        
        # Yakın mesafedeki engelleri tespit et
        close_obstacles = roi < 1.5  # 1.5 metre yakınındaki engeller
        
        if np.any(close_obstacles):
            # Engel varsa haritaya ekle
            obstacle_positions = np.where(close_obstacles)
            for i in range(len(obstacle_positions[0])):
                # Kamera koordinatlarından harita koordinatlarına dönüştür
                cam_x = obstacle_positions[1][i] - width // 2
                cam_y = obstacle_positions[0][i]
                
                # Basit projeksiyon (kamera kalibrasyonu gerektirebilir)
                world_x = cam_x * 0.01  # Ölçekleme faktörü
                world_y = cam_y * 0.01 + 1.0  # İleri yön
                
                map_x = int(self.robot_x + world_x / self.map_resolution)
                map_y = int(self.robot_y + world_y / self.map_resolution)
                
                if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                    self.obstacle_map[map_y, map_x] = 100

    def publish_obstacle_map(self):
        """Engel haritasını yayınla"""
        # Derinlik kamerası verisi ile ek engel tespiti
        self.process_depth_obstacles()
        
        # OccupancyGrid mesajı oluştur
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'
        
        # Harita bilgileri
        grid_msg.info.width = self.map_width
        grid_msg.info.height = self.map_height
        grid_msg.info.resolution = self.map_resolution
        
        # Harita orijini (robot merkezi)
        grid_msg.info.origin.position.x = -self.robot_x * self.map_resolution
        grid_msg.info.origin.position.y = -self.robot_y * self.map_resolution
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        
        # Harita verisi
        grid_msg.data = self.obstacle_map.flatten().tolist()
        
        self.obstacle_map_pub.publish(grid_msg)
        self.get_logger().debug("Engel haritası yayınlandı")


def main(args=None):
    rclpy.init(args=args)
    
    node = ObstacleDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Engel Tespiti Node durduruluyor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()