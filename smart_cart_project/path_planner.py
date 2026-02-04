import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.subscription = self.create_subscription(String, '/selected_menu', self.menu_callback, 10)
        self.get_logger().info('경로 플래너 대기 중...')

    def menu_callback(self, msg):
        # 여기서 TSP 알고리즘 등을 통해 최적 좌표 순서를 계산함
        self.get_logger().info(f'메뉴 수신: {msg.data} -> 최적 경로 계산 시작')

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()