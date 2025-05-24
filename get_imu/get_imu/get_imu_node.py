import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import socket
import threading

HOST = '127.0.0.1'
PORT = 6006

class IMUNode(Node):
    def __init__(self):
        super().__init__('get_imu')
        self.publisher_ = self.create_publisher(Imu, '/get_imu/data', 10)
        self.latest_accel = Vector3()
        self.latest_gyro = Vector3()
        self.lock = threading.Lock()
        
        self.tcp_thread = threading.Thread(target=self.tcp_server, daemon=True)
        self.tcp_thread.start()
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info("IMU node başlatıldı")

    def tcp_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            server.bind((HOST, PORT))
            server.listen(1)
            self.get_logger().info(f"IMU TCP server listening on {HOST}:{PORT}")
            
            while rclpy.ok():
                try:
                    conn, addr = server.accept()
                    data = conn.recv(1024).decode('utf-8').strip()
                    conn.close()
                    
                    if data:
                        parts = data.split(',')
                        if len(parts) >= 6:
                            with self.lock:
                                self.latest_accel.x = float(parts[0])
                                self.latest_accel.y = float(parts[1])
                                self.latest_accel.z = float(parts[2])
                                self.latest_gyro.x = float(parts[3])
                                self.latest_gyro.y = float(parts[4])
                                self.latest_gyro.z = float(parts[5])
                                
                except Exception as e:
                    self.get_logger().error(f"IMU TCP error: {e}")
                    
        except Exception as e:
            self.get_logger().error(f"IMU TCP server error: {e}")

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        with self.lock:
            msg.linear_acceleration = self.latest_accel
            msg.angular_velocity = self.latest_gyro
            
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
