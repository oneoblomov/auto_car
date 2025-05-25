import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import socket
import numpy as np
import cv2

HOST = '127.0.0.1'
PORT = 8008
class ImageReceiverNode(Node):
    def __init__(self):
        super().__init__('get_targ_cam_node')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((HOST, PORT))
        self.sock.listen(1)
        self.get_logger().info(f'Görüntü alımı için {HOST}:{PORT} portu dinleniyor...')

    def receive_and_publish(self):
        while rclpy.ok():
            self.get_logger().info('Unity bağlantısı bekleniyor...')
            conn, _ = self.sock.accept()
            self.get_logger().info('Unity bağlandı!')
            try:
                while rclpy.ok():
                    # 4 byte uzunluk oku
                    length_bytes = conn.recv(4)
                    if not length_bytes:
                        break
                    length = int.from_bytes(length_bytes, 'little')
                    # Görüntü verisini al
                    data = b''
                    while len(data) < length:
                        packet = conn.recv(length - len(data))
                        if not packet:
                            break
                        data += packet
                    if len(data) < length:
                        break
                    # JPEG'den numpy array'e
                    np_arr = np.frombuffer(data, np.uint8)
                    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    if img is None:
                        self.get_logger().warn('Görüntü decode edilemedi!')
                        continue
                    # sensor_msgs/Image mesajı oluştur
                    msg = Image()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'camera_link'
                    msg.height, msg.width, _ = img.shape
                    msg.encoding = 'bgr8'
                    msg.data = img.tobytes()
                    msg.step = msg.width * 3
                    self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Bağlantı hatası: {e}')
            finally:
                conn.close()

def main(args=None):
    rclpy.init(args=args)
    node = ImageReceiverNode()
    try:
        node.receive_and_publish()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()