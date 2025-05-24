# ROS2 Temel Kurulum Adımları

## 1. Workspace ve Paket Oluşturma

```bash
# Workspace oluştur
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash

# Örnek: get_lidar paketi oluştur (Python)
cd src
ros2 pkg create --build-type ament_python get_lidar
# Diğer node'lar için de aynı şekilde paket oluşturun
```

## 2. Custom Mesaj/Servis/Action Tanımları

```bash
# Mesaj paketi oluştur
ros2 pkg create --build-type ament_cmake custom_msgs
ros2 pkg create --build-type ament_cmake custom_srvs
ros2 pkg create --build-type ament_cmake custom_actions

# msg, srv, action dosyalarını ilgili paketlerin 'msg', 'srv', 'action' klasörlerine ekleyin
# Örnek: custom_msgs/msg/SignArray.msg
```

## 3. Node Kodlarını Yazma

Her paketin içinde, örneğin `get_lidar/get_lidar_node.py` gibi dosyalar oluşturun ve ROS2 node kodunuzu yazın.

```python
# get_lidar/get_lidar_node.py (örnek Python node)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarNode(Node):
    def __init__(self):
        super().__init__('get_lidar')
        self.publisher_ = self.create_publisher(LaserScan, '/get_lidar/scan', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        msg = LaserScan()
        # Dummy data for demonstration
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'
        msg.angle_min = -1.57
        msg.angle_max = 1.57
        msg.angle_increment = 0.01
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.12
        msg.range_max = 3.5
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

```

## 4. Launch Dosyası ile Başlatma

Tüm node'ları başlatmak için bir launch dosyası yazın.

```python
# launch/all_nodes.launch.py (örnek)
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='get_lidar', executable='get_lidar_node', name='get_lidar'),
        Node(package='detect_sign', executable='detect_sign_node', name='detect_sign'),
        # ...diğer node'lar...
    ])
```

## 5. Derleme ve Çalıştırma

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch <paket_adı> <launch_dosyası>
# Örnek: ros2 launch get_lidar all_nodes.launch.py
```