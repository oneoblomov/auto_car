# get_encoder/get_encoder_node.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import socket
import threading

HOST = '127.0.0.1'
PORT = 7007

class EncoderNode(Node):
    def __init__(self):
        super().__init__('get_encoder')
        self.publisher_ = self.create_publisher(Odometry, '/get_encoder/odom', 10)
        self.left = 0.0
        self.right = 0.0
        self.lock = threading.Lock()
        threading.Thread(target=self.tcp_server, daemon=True).start()
        self.create_timer(0.1, self.timer_callback)

    def tcp_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, PORT))
        server.listen(1)
        while rclpy.ok():
            try:
                conn, _ = server.accept()
                try:
                    data = conn.recv(1024).decode('utf-8').strip()
                    if data:
                        sol, sag = map(float, data.split(','))
                        with self.lock:
                            self.left = sol
                            self.right = sag
                except Exception as e:
                    self.get_logger().error(f"Parse error: {e}")
                finally:
                    conn.close()
            except Exception as e:
                self.get_logger().error(f"TCP connection error: {e}")

    def timer_callback(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        with self.lock:
            # Basit örnek: toplam dönüşü pozisyona çevir
            msg.pose.pose.position.x = (self.left + self.right) / 2.0
            msg.pose.pose.position.y = 0.0
            msg.pose.pose.position.z = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()