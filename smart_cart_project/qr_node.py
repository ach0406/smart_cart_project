import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from pyzbar import pyzbar

class QRNode(Node):
    def __init__(self):
        super().__init__('qr_node')
        self.publisher_ = self.create_publisher(String, '/selected_menu', 10)
        self.cap = cv2.VideoCapture(0) # USB 카메라
        self.timer = self.create_timer(0.1, self.scan_qr)

    def scan_qr(self):
        ret, frame = self.cap.read()
        if ret:
            decoded_objs = pyzbar.decode(frame)
            for obj in decoded_objs:
                qr_data = obj.data.decode('utf-8')
                self.get_logger().info(f'QR 인식됨: {qr_data}')
                msg = String()
                msg.data = qr_data
                self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = QRNode()
    rclpy.spin(node)
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()