import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import socket
import numpy as np
import cv2
from cv_bridge import CvBridge

class ImageSocketServer(Node):
    def __init__(self):
        super().__init__('image_socket_server')
        
        # Подписка на топик с изображением
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/pure_image',
            self.image_callback,
            10
        )
        self.subscription  # Чтобы избежать предупреждения
        
        # Инициализация CV Bridge
        self.bridge = CvBridge()
        
        # Настройка серверного сокета
        self.socket_host = '10.0.2.2'  # Локальный хост
        self.socket_port = 8081
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.socket_host, self.socket_port))
        self.server_socket.listen(1)  # Ожидаем 1 подключение
        
        self.get_logger().info(f"Server started on {self.socket_host}:{self.socket_port}")
        self.client_socket = None
        
        # Запускаем отдельный поток для принятия подключений
        import threading
        self.connection_thread = threading.Thread(target=self.accept_connections)
        self.connection_thread.start()
    
    def accept_connections(self):
        while rclpy.ok():
            try:
                self.client_socket, addr = self.server_socket.accept()
                self.get_logger().info(f"Client connected: {addr}")
            except Exception as e:
                self.get_logger().error(f"Connection error: {e}")
                break
    
    def image_callback(self, msg):
        if self.client_socket is None:
            return
            
        try:
            # Конвертируем ROS Image в OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Кодируем в JPEG (для depth-изображений может потребоваться нормализация)
            _, img_encoded = cv2.imencode('.jpg', cv_image)
            
            # Отправляем размер и данные
            img_size = len(img_encoded).to_bytes(4, byteorder='big')
            self.client_socket.sendall(img_size + img_encoded.tobytes())
            
        except Exception as e:
            self.get_logger().error(f"Error sending image: {e}")
            try:
                self.client_socket.close()
            except:
                pass
            self.client_socket = None

    def destroy_node(self):
        self.server_socket.close()
        if self.client_socket:
            self.client_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSocketServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()