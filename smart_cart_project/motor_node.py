import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.get_logger().info('🚗 모터 노드 준비 완료!')

    def cmd_callback(self, msg):
        left = msg.linear.x - msg.angular.z
        right = msg.linear.x + msg.angular.z
        self.get_logger().info(f'Motors -> L: {left:.2f}, R: {right:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
