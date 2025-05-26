#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import yaml
import os

class MapServerNode(Node):
    def __init__(self):
        super().__init__('map_server')
        
        # Publishers
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            10
        )
        
        # Harita parametreleri
        self.map_width = 400  # piksel
        self.map_height = 400  # piksel
        self.map_resolution = 0.1  # metre/piksel (10cm/piksel)
        self.map_origin_x = -20.0  # metre
        self.map_origin_y = -20.0  # metre
        
        # Harita verisi
        # Harita dosyasından yükle (varsa), yoksa varsayılan harita
        yaml_path = self.declare_parameter('map_yaml_file', '').get_parameter_value().string_value
        if yaml_path and os.path.exists(yaml_path):
            self.static_map, self.map_width, self.map_height, self.map_resolution, self.map_origin_x, self.map_origin_y = self.load_map_from_file(yaml_path)
            self.get_logger().info(f"YAML'den harita yüklendi: {yaml_path}")
        else:
            self.static_map = self.create_default_map()
        
        # Timer - haritayı düzenli olarak yayınla
        self.timer = self.create_timer(1.0, self.publish_map)  # 1 Hz
        
        self.get_logger().info("Map Server başlatıldı")
    
    def create_default_map(self):
        """Varsayılan harita oluştur"""
        # Boş harita (tüm alanlar serbest)
        map_data = np.full((self.map_height, self.map_width), -1, dtype=np.int8)
        
        # Sınır duvarları ekle
        map_data[0, :] = 100  # Üst sınır
        map_data[-1, :] = 100  # Alt sınır
        map_data[:, 0] = 100  # Sol sınır
        map_data[:, -1] = 100  # Sağ sınır
        
        # Örnek iç duvarlar ve engeller
        # Merkezi dikdörtgen engel
        map_data[180:220, 180:220] = 100
        
        # L şeklinde duvar
        map_data[100:150, 100:120] = 100
        map_data[130:180, 100:120] = 100
        
        # Koridor oluşturmak için paralel duvarlar
        map_data[250:270, 50:150] = 100
        map_data[290:310, 50:150] = 100
        
        # Maze benzeri yapı
        map_data[50:100, 250:270] = 100
        map_data[80:130, 270:290] = 100
        map_data[110:160, 250:270] = 100
        
        # Serbest alanları belirgin yap
        # Merkez alan
        map_data[190:210, 190:210] = 0
        
        # Ana koridor
        map_data[270:290, 50:150] = 0
        
        # Geniş açık alanlar
        map_data[50:100, 50:100] = 0
        map_data[300:350, 300:350] = 0
        
        return map_data
    
    def load_map_from_file(self, yaml_file_path):
        """YAML dosyasından harita yükle (PGM okuma dahil)"""
        try:
            with open(yaml_file_path, 'r') as file:
                map_yaml = yaml.safe_load(file)
            pgm_file = os.path.join(os.path.dirname(yaml_file_path), map_yaml['image'])
            resolution = float(map_yaml['resolution'])
            origin = map_yaml['origin']  # [x, y, theta]
            # PGM dosyasını oku
            with open(pgm_file, 'rb') as f:
                header = f.readline()
                if b'P5' not in header:
                    raise Exception('PGM formatı P5 olmalı!')
                # Yorum satırlarını atla
                while True:
                    line = f.readline()
                    if line[0] != ord('#'):
                        break
                width_height = line.strip().split()
                width, height = int(width_height[0]), int(width_height[1])
                maxval = int(f.readline())
                img = np.frombuffer(f.read(), dtype=np.uint8).reshape((height, width))
            # ROS harita formatına çevir (0: serbest, 100: engel, -1: bilinmiyor)
            map_data = np.zeros_like(img, dtype=np.int8)
            map_data[img == 0] = 100  # Engel (siyah)
            map_data[img == 254] = 0  # Serbest (beyaz)
            map_data[(img != 0) & (img != 254)] = -1  # Bilinmiyor (gri)
            return map_data, width, height, resolution, origin[0], origin[1]
        except Exception as e:
            self.get_logger().error(f"Harita yüklenemedi: {e}")
            return self.create_default_map(), self.map_width, self.map_height, self.map_resolution, self.map_origin_x, self.map_origin_y
    
    def publish_map(self):
        """Haritayı yayınla"""
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'
        
        # Harita bilgileri
        grid_msg.info.width = self.map_width
        grid_msg.info.height = self.map_height
        grid_msg.info.resolution = self.map_resolution
        
        # Harita orijini
        grid_msg.info.origin.position.x = self.map_origin_x
        grid_msg.info.origin.position.y = self.map_origin_y
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        
        # Harita verisi (numpy array'i list'e çevir)
        grid_msg.data = self.static_map.flatten().tolist()
        
        self.map_pub.publish(grid_msg)
        self.get_logger().debug("Statik harita yayınlandı")

def main(args=None):
    rclpy.init(args=args)
    
    node = MapServerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Map Server durduruluyor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
