#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from typing import Tuple, List


class DWAPathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        # Subscribers
        self.obstacle_map_sub = self.create_subscription(
            OccupancyGrid,
            '/detect_obstacle/obstacle_map',
            self.obstacle_map_callback,
            10
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/get_lidar/scan',
            self.lidar_callback,
            10
        )
        
        # DWA parametreleri
        self.max_speed = 2.0  # m/s
        self.min_speed = -1.0  # m/s
        self.max_yaw_rate = 40.0 * math.pi / 180.0  # rad/s
        self.max_accel = 0.2  # m/ss
        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # rad/ss
        self.v_resolution = 0.01  # m/s
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # rad/s
        self.dt = 0.1  # zaman adımı
        self.predict_time = 3.0  # tahmin süresi
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001
        
        # Robot durumu
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_vx = 0.0
        self.robot_vy = 0.0
        self.robot_omega = 0.0
        
        # Hedef
        self.goal_x = None
        self.goal_y = None
        
        # Engel haritası
        self.obstacle_map = None
        self.map_resolution = 0.1
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        
        # LiDAR verileri
        self.latest_lidar = None
        
        # Timer
        self.timer = self.create_timer(0.1, self.planning_loop)  # 10 Hz
        
        self.get_logger().info("DWA Path Planner başlatıldı")

    def obstacle_map_callback(self, msg):
        """Engel haritası callback"""
        self.obstacle_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

    def goal_callback(self, msg):
        """Hedef callback"""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.get_logger().info(f"Yeni hedef: ({self.goal_x:.2f}, {self.goal_y:.2f})")

    def lidar_callback(self, msg):
        """LiDAR callback"""
        self.latest_lidar = msg

    def planning_loop(self):
        """Ana planlama döngüsü"""
        if self.goal_x is None or self.goal_y is None:
            return
            
        # Hedefe ulaştık mı kontrol et
        distance_to_goal = math.sqrt((self.goal_x - self.robot_x)**2 + (self.goal_y - self.robot_y)**2)
        if distance_to_goal < 0.5:  # 50 cm yakınsa dur
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            self.get_logger().info("Hedefe ulaşıldı!")
            return
        
        # DWA ile en iyi hız komutunu hesapla
        best_v, best_omega = self.dwa_planning()
        
        # Komut gönder
        cmd_vel = Twist()
        cmd_vel.linear.x = best_v
        cmd_vel.angular.z = best_omega
        self.cmd_vel_pub.publish(cmd_vel)

    def dwa_planning(self) -> Tuple[float, float]:
        """Dynamic Window Approach ile yol planlama"""
        # Dynamic window hesapla
        dw = self.calc_dynamic_window()
        
        # En iyi trajectory'yi bul
        best_v = 0.0
        best_omega = 0.0
        best_cost = float('inf')
        best_trajectory = None
        
        for v in np.arange(dw[0], dw[1], self.v_resolution):
            for omega in np.arange(dw[2], dw[3], self.yaw_rate_resolution):
                # Trajectory hesapla
                trajectory = self.predict_trajectory(v, omega)
                
                # Maliyeti hesapla
                cost = self.calc_cost(trajectory, v, omega)
                
                if cost < best_cost:
                    best_cost = cost
                    best_v = v
                    best_omega = omega
                    best_trajectory = trajectory
        
        # En iyi yolu yayınla
        if best_trajectory is not None:
            self.publish_path(best_trajectory)
        
        return best_v, best_omega

    def calc_dynamic_window(self) -> List[float]:
        """Dynamic window hesapla"""
        # Hız limitleri
        vs = [self.min_speed, self.max_speed,
              -self.max_yaw_rate, self.max_yaw_rate]
        
        # İvme limitleri
        vd = [self.robot_vx - self.max_accel * self.dt,
              self.robot_vx + self.max_accel * self.dt,
              self.robot_omega - self.max_delta_yaw_rate * self.dt,
              self.robot_omega + self.max_delta_yaw_rate * self.dt]
        
        # Kesişim
        dw = [max(vs[0], vd[0]), min(vs[1], vd[1]),
              max(vs[2], vd[2]), min(vs[3], vd[3])]
        
        return dw

    def predict_trajectory(self, v: float, omega: float) -> np.ndarray:
        """Trajectory tahmin et"""
        trajectory = np.array([[self.robot_x, self.robot_y, self.robot_yaw, v, omega]])
        
        x, y, yaw = self.robot_x, self.robot_y, self.robot_yaw
        
        time = 0
        while time <= self.predict_time:
            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            yaw += omega * self.dt
            trajectory = np.vstack((trajectory, [x, y, yaw, v, omega]))
            time += self.dt
        
        return trajectory

    def calc_cost(self, trajectory: np.ndarray, v: float, omega: float) -> float:
        """Trajectory maliyetini hesapla"""
        # Engel maliyeti
        obstacle_cost = self.calc_obstacle_cost(trajectory)
        if obstacle_cost == float('inf'):
            return float('inf')
        
        # Hedefe uzaklık maliyeti
        goal_cost = self.calc_to_goal_cost(trajectory)
        
        # Hız maliyeti
        speed_cost = self.speed_cost_gain * (self.max_speed - v)
        
        total_cost = (self.to_goal_cost_gain * goal_cost + 
                     self.speed_cost_gain * speed_cost + 
                     self.obstacle_cost_gain * obstacle_cost)
        
        return total_cost

    def calc_obstacle_cost(self, trajectory: np.ndarray) -> float:
        """Engel maliyetini hesapla"""
        if self.latest_lidar is None:
            return 0.0
        
        min_distance = float('inf')
        
        for point in trajectory:
            x, y = point[0], point[1]
            
            # LiDAR ile mesafe kontrol et
            for i, range_val in enumerate(self.latest_lidar.ranges):
                if (range_val < self.latest_lidar.range_min or 
                    range_val > self.latest_lidar.range_max):
                    continue
                
                angle = self.latest_lidar.angle_min + i * self.latest_lidar.angle_increment
                obstacle_x = self.robot_x + range_val * math.cos(angle + self.robot_yaw)
                obstacle_y = self.robot_y + range_val * math.sin(angle + self.robot_yaw)
                
                distance = math.sqrt((x - obstacle_x)**2 + (y - obstacle_y)**2)
                min_distance = min(min_distance, distance)
        
        if min_distance < 0.3:  # 30 cm yakınsa
            return float('inf')
        
        return 1.0 / min_distance if min_distance < 2.0 else 0.0

    def calc_to_goal_cost(self, trajectory: np.ndarray) -> float:
        """Hedefe uzaklık maliyetini hesapla"""
        dx = self.goal_x - trajectory[-1, 0]
        dy = self.goal_y - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))
        
        return cost

    def publish_path(self, trajectory: np.ndarray):
        """Planlanan yolu yayınla"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        poses_list = []
        for point in trajectory:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            
            # Yaw açısından quaternion hesapla
            pose.pose.orientation.w = math.cos(point[2] / 2.0)
            pose.pose.orientation.z = math.sin(point[2] / 2.0)
            
            poses_list.append(pose)
        
        path_msg.poses = poses_list
        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = DWAPathPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Path Planner Node durduruluyor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
