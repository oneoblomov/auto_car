#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math
from scipy.spatial.distance import cdist

class SLAMNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        
        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/slam_map', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/slam_pose', 10)
        
        # Subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/get_lidar/scan',
            self.lidar_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/get_encoder/odom',
            self.odom_callback,
            10
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # SLAM parametreleri
        self.map_width = 500
        self.map_height = 500
        self.map_resolution = 0.1  # 10cm/piksel
        self.map_origin_x = -25.0
        self.map_origin_y = -25.0
        
        # SLAM haritası - log-odds formatında
        self.log_odds_map = np.zeros((self.map_height, self.map_width), dtype=np.float32)
        
        # Robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # Particle filter için
        self.num_particles = 100
        self.particles = self.initialize_particles()
        self.particle_weights = np.ones(self.num_particles) / self.num_particles
        
        # Landmark storage
        self.landmarks = []
        
        # Timer
        self.timer = self.create_timer(0.5, self.publish_slam_results)  # 2 Hz
        
        self.get_logger().info("SLAM Node başlatıldı")
    
    def initialize_particles(self):
        """Particle filter için parçacıkları başlat"""
        particles = np.zeros((self.num_particles, 3))  # [x, y, yaw]
        # Başlangıçta robot pozisyonu etrafında dağıt
        particles[:, 0] = np.random.normal(0, 0.5, self.num_particles)  # x
        particles[:, 1] = np.random.normal(0, 0.5, self.num_particles)  # y
        particles[:, 2] = np.random.normal(0, 0.1, self.num_particles)  # yaw
        return particles
    
    def lidar_callback(self, msg):
        """LiDAR verisi ile SLAM güncelleme"""
        # Basit occupancy grid mapping
        self.update_occupancy_grid(msg)
        
        # Particle filter ile lokalizasyon
        self.update_particles(msg)
        
        # En iyi parçacığı robot pozisyonu olarak al
        best_particle_idx = np.argmax(self.particle_weights)
        self.robot_x = self.particles[best_particle_idx, 0]
        self.robot_y = self.particles[best_particle_idx, 1]
        self.robot_yaw = self.particles[best_particle_idx, 2]
    
    def odom_callback(self, msg):
        """Odometry ile parçacıkları hareket ettir"""
        # Odometry bilgisini al
        dx = msg.twist.twist.linear.x * 0.1  # 10Hz varsayarak
        dy = msg.twist.twist.linear.y * 0.1
        dyaw = msg.twist.twist.angular.z * 0.1
        
        # Tüm parçacıkları hareket ettir (noise ekleyerek)
        for i in range(self.num_particles):
            # Noise ekle
            noise_x = np.random.normal(0, 0.1)
            noise_y = np.random.normal(0, 0.1)
            noise_yaw = np.random.normal(0, 0.05)
            
            # Hareket modeli uygula
            self.particles[i, 0] += (dx + noise_x) * math.cos(self.particles[i, 2])
            self.particles[i, 1] += (dy + noise_y) * math.sin(self.particles[i, 2])
            self.particles[i, 2] += dyaw + noise_yaw
    
    def update_occupancy_grid(self, lidar_msg):
        """LiDAR verisi ile occupancy grid güncelle"""
        angle = lidar_msg.angle_min
        
        for i, range_val in enumerate(lidar_msg.ranges):
            if range_val < lidar_msg.range_min or range_val > lidar_msg.range_max:
                angle += lidar_msg.angle_increment
                continue
            
            # Global koordinatlara çevir
            global_x = self.robot_x + range_val * math.cos(angle + self.robot_yaw)
            global_y = self.robot_y + range_val * math.sin(angle + self.robot_yaw)
            
            # Harita koordinatlarına çevir
            map_x = int((global_x - self.map_origin_x) / self.map_resolution)
            map_y = int((global_y - self.map_origin_y) / self.map_resolution)
            
            # Harita sınırları içinde mi kontrol et
            if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                # Log-odds güncelleme (Bayesian)
                self.log_odds_map[map_y, map_x] += 0.9  # Engel olasılığını artır
                
                # Serbest alanları güncelle (ray casting)
                self.update_free_space(
                    int((self.robot_x - self.map_origin_x) / self.map_resolution),
                    int((self.robot_y - self.map_origin_y) / self.map_resolution),
                    map_x, map_y
                )
            
            angle += lidar_msg.angle_increment
    
    def update_free_space(self, x0, y0, x1, y1):
        """Bresenham çizgi algoritması ile serbest alanları güncelle"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                # Serbest alan olasılığını artır (hedefe 90% mesafede)
                distance_ratio = math.sqrt((x-x0)**2 + (y-y0)**2) / math.sqrt((x1-x0)**2 + (y1-y0)**2)
                if distance_ratio < 0.9:  # Hedefe %90 mesafede serbest alan
                    self.log_odds_map[y, x] -= 0.3
            
            if x == x1 and y == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
    
    def update_particles(self, lidar_msg):
        """Particle filter güncelleme"""
        # Her parçacık için likelihood hesapla
        for i in range(self.num_particles):
            likelihood = self.calculate_particle_likelihood(
                self.particles[i], lidar_msg
            )
            self.particle_weights[i] *= likelihood
        
        # Ağırlıkları normalize et
        self.particle_weights /= np.sum(self.particle_weights)
        
        # Effective sample size kontrol et
        eff_sample_size = 1.0 / np.sum(self.particle_weights ** 2)
        if eff_sample_size < self.num_particles / 2:
            self.resample_particles()
    
    def calculate_particle_likelihood(self, particle, lidar_msg):
        """Parçacık likelihood hesaplama"""
        likelihood = 1.0
        angle = lidar_msg.angle_min
        
        # Her birkaç LiDAR noktasını kontrol et (performans için)
        step = max(1, len(lidar_msg.ranges) // 20)
        
        for i in range(0, len(lidar_msg.ranges), step):
            range_val = lidar_msg.ranges[i]
            if range_val < lidar_msg.range_min or range_val > lidar_msg.range_max:
                angle += lidar_msg.angle_increment * step
                continue
            
            # Parçacık pozisyonundan beklenen mesafe
            expected_range = self.get_expected_range(particle, angle)
            
            # Gauss dağılımı ile likelihood
            diff = abs(range_val - expected_range)
            likelihood *= math.exp(-diff**2 / (2 * 0.1**2))  # sigma = 0.1
            
            angle += lidar_msg.angle_increment * step
        
        return likelihood
    
    def get_expected_range(self, particle, angle):
        """Parçacık pozisyonundan beklenen LiDAR mesafesi"""
        # Basit implementasyon - mevcut haritadan tahmin
        max_range = 3.5
        
        # Ray casting ile engele çarpana kadar git
        for r in np.arange(0.1, max_range, 0.1):
            x = particle[0] + r * math.cos(angle + particle[2])
            y = particle[1] + r * math.sin(angle + particle[2])
            
            # Harita koordinatlarına çevir
            map_x = int((x - self.map_origin_x) / self.map_resolution)
            map_y = int((y - self.map_origin_y) / self.map_resolution)
            
            if (0 <= map_x < self.map_width and 
                0 <= map_y < self.map_height and
                self.log_odds_map[map_y, map_x] > 0.5):  # Engel
                return r
        
        return max_range
    
    def resample_particles(self):
        """Systematic resampling"""
        # Cumulative sum
        cumsum = np.cumsum(self.particle_weights)
        
        # Yeni parçacıklar
        new_particles = np.zeros_like(self.particles)
        
        # Systematic sampling
        start = np.random.uniform(0, 1.0/self.num_particles)
        
        j = 0
        for i in range(self.num_particles):
            u = start + i / self.num_particles
            while u > cumsum[j]:
                j += 1
            new_particles[i] = self.particles[j]
            
            # Küçük noise ekle
            new_particles[i, 0] += np.random.normal(0, 0.1)
            new_particles[i, 1] += np.random.normal(0, 0.1)
            new_particles[i, 2] += np.random.normal(0, 0.05)
        
        self.particles = new_particles
        self.particle_weights = np.ones(self.num_particles) / self.num_particles
    
    def publish_slam_results(self):
        """SLAM sonuçlarını yayınla"""
        # Occupancy grid yayınla
        self.publish_occupancy_grid()
        
        # Robot pose yayınla
        self.publish_robot_pose()
        
        # TF transform yayınla
        self.publish_transform()
    
    def publish_occupancy_grid(self):
        """SLAM haritasını OccupancyGrid olarak yayınla"""
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'
        
        grid_msg.info.width = self.map_width
        grid_msg.info.height = self.map_height
        grid_msg.info.resolution = self.map_resolution
        grid_msg.info.origin.position.x = self.map_origin_x
        grid_msg.info.origin.position.y = self.map_origin_y
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        
        # Log-odds'ı occupancy probability'e çevir
        prob_map = 1.0 / (1.0 + np.exp(-self.log_odds_map))
        
        # ROS occupancy format'ına çevir (-1: unknown, 0-100: probability)
        occupancy_map = np.full_like(prob_map, -1, dtype=np.int8)
        
        # Bilinen alanları dönüştür
        known_mask = np.abs(self.log_odds_map) > 0.1
        occupancy_map[known_mask] = (prob_map[known_mask] * 100).astype(np.int8)
        
        grid_msg.data = occupancy_map.flatten().tolist()
        self.map_pub.publish(grid_msg)
    
    def publish_robot_pose(self):
        """Robot pozisyonunu yayınla"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        pose_msg.pose.position.x = self.robot_x
        pose_msg.pose.position.y = self.robot_y
        pose_msg.pose.position.z = 0.0
        
        # Yaw'dan quaternion hesapla
        pose_msg.pose.orientation.w = math.cos(self.robot_yaw / 2.0)
        pose_msg.pose.orientation.z = math.sin(self.robot_yaw / 2.0)
        
        self.pose_pub.publish(pose_msg)
    
    def publish_transform(self):
        """Map -> base_link transform yayınla"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.robot_x
        t.transform.translation.y = self.robot_y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.w = math.cos(self.robot_yaw / 2.0)
        t.transform.rotation.z = math.sin(self.robot_yaw / 2.0)
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    
    node = SLAMNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("SLAM Node durduruluyor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
