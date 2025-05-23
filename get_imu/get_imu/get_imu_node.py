import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import socket
import threading

class ImuNode(Node):
    def __init__(self):
        super().__init__('get_imu')
        self.publisher_ = self.create_publisher(Imu, '/get_imu/data', 10)
        self.imu_data = [0.0] * 6  # ax, ay, az, gx, gy, gz
        self.lock = threading.Lock()
        self.tcp_thread = threading.Thread(target=self.tcp_server, daemon=True)
        self.tcp_thread.start()
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

    def tcp_server(self):
        HOST = '127.0.0.1'
        PORT = 6006
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, PORT))
        server.listen(1)
        self.get_logger().info(f"IMU TCP server listening on {HOST}:{PORT}")
        while rclpy.ok():
            conn, addr = server.accept()
            try:
                buffer = b''
                while rclpy.ok():
                    chunk = conn.recv(1024)
                    if not chunk:
                        break
                    buffer += chunk
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        data_str = line.decode('utf-8').strip()
                        if data_str:
                            parts = data_str.split(',')
                            if len(parts) >= 6:
                                with self.lock:
                                    self.imu_data = [float(x) for x in parts[:6]]
            except Exception as e:
                self.get_logger().error(f"IMU TCP error: {e}")
            finally:
                conn.close()
                
    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        with self.lock:
            ax, ay, az, gx, gy, gz = self.imu_data
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        # Orientation bilinmiyor, sıfır bırakılıyor
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()