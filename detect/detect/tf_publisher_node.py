#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math


class TFPublisherNode(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/get_encoder/odom',
            self.odom_callback,
            10
        )
        
        # Robot pozisyonu
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # Timer for static transforms
        self.timer = self.create_timer(0.1, self.publish_static_transforms)  # 10 Hz
        
        self.get_logger().info("TF Publisher Node başlatıldı")

    def odom_callback(self, msg):
        """Odometry callback - robot pozisyonunu güncelle ve TF yayınla"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Quaternion'dan yaw açısını hesapla
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # map -> odom transform
        self.publish_map_to_odom()
        
        # odom -> base_link transform
        self.publish_odom_to_base_link(msg)

    def publish_map_to_odom(self):
        """map -> odom transform yayınla (SLAM olmadığı için identity)"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        
        # Identity transform (SLAM yoksa map = odom)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)

    def publish_odom_to_base_link(self, odom_msg):
        """odom -> base_link transform yayınla"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Robot pozisyonunu kullan
        t.transform.translation.x = self.robot_x
        t.transform.translation.y = self.robot_y
        t.transform.translation.z = 0.0
        
        # Quaternion'u direkt kopyala
        t.transform.rotation.x = odom_msg.pose.pose.orientation.x
        t.transform.rotation.y = odom_msg.pose.pose.orientation.y
        t.transform.rotation.z = odom_msg.pose.pose.orientation.z
        t.transform.rotation.w = odom_msg.pose.pose.orientation.w
        
        self.tf_broadcaster.sendTransform(t)

    def publish_static_transforms(self):
        """Statik transform'ları sürekli yayınla (robot_state_publisher olmadığı için)"""
        current_time = self.get_clock().now().to_msg()
        
        # base_link -> lidar_link
        t_lidar = TransformStamped()
        t_lidar.header.stamp = current_time
        t_lidar.header.frame_id = 'base_link'
        t_lidar.child_frame_id = 'lidar_link'
        t_lidar.transform.translation.x = 0.63
        t_lidar.transform.translation.y = 0.0
        t_lidar.transform.translation.z = 0.26
        t_lidar.transform.rotation.w = 1.0
        
        # base_link -> imu_link
        t_imu = TransformStamped()
        t_imu.header.stamp = current_time
        t_imu.header.frame_id = 'base_link'
        t_imu.child_frame_id = 'imu_link'
        t_imu.transform.translation.x = 0.0
        t_imu.transform.translation.y = 0.0
        t_imu.transform.translation.z = 0.3
        t_imu.transform.rotation.w = 1.0
        
        # base_link -> camera_link
        t_camera = TransformStamped()
        t_camera.header.stamp = current_time
        t_camera.header.frame_id = 'base_link'
        t_camera.child_frame_id = 'camera_link'
        t_camera.transform.translation.x = 0.6
        t_camera.transform.translation.y = 0.05
        t_camera.transform.translation.z = 0.28
        t_camera.transform.rotation.w = 1.0
        
        # base_link -> back_camera_link
        t_back_camera = TransformStamped()
        t_back_camera.header.stamp = current_time
        t_back_camera.header.frame_id = 'base_link'
        t_back_camera.child_frame_id = 'back_camera_link'
        t_back_camera.transform.translation.x = -0.6
        t_back_camera.transform.translation.y = 0.0
        t_back_camera.transform.translation.z = 0.28
        # 180 derece dönüş (geri kamera)
        t_back_camera.transform.rotation.x = 0.0
        t_back_camera.transform.rotation.y = 0.0
        t_back_camera.transform.rotation.z = 1.0
        t_back_camera.transform.rotation.w = 0.0
        
        # Tüm transform'ları yayınla
        transforms = [t_lidar, t_imu, t_camera, t_back_camera]
        for transform in transforms:
            self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    
    node = TFPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("TF Publisher Node durduruluyor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
