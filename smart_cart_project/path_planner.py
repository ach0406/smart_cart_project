import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist # 속도 명령을 보내기 위해 추가

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        # 1. 구독자(Sub) 설정: 메뉴 선택과 제스처 명령을 듣습니다.
        self.menu_sub = self.create_subscription(String, '/selected_menu', self.menu_callback, 10)
        self.cmd_sub = self.create_subscription(String, '/cart_command', self.command_callback, 10)
        
        # 2. 발행자(Pub) 설정: 모터 노드에 직접 명령을 내릴 수도 있습니다.
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 3. 로봇의 상태 관리 변수 (핵심!)
        self.is_paused = True  # 처음에는 멈춤 상태
        self.current_goal = None
        
        self.get_logger().info('경로 플래너가 대기 중입니다. 제스처를 보여주세요!')

    def menu_callback(self, msg):
        self.current_goal = msg.data
        self.get_logger().info(f'목적지 수신: {self.current_goal}. 주행 재개 제스처를 기다립니다.')

    def command_callback(self, msg):
        if msg.data == "RESUME":
            if self.current_goal is not None:
                self.is_paused = False # 일시정지 해제!
                self.get_logger().info(f'✋ 제스처 확인! {self.current_goal}(으)로 출발합니다.')
                self.start_moving()
            else:
                self.get_logger().warn('목적지가 설정되지 않았습니다. 메뉴를 먼저 골라주세요.')

    def start_moving(self):
        # 실제로는 여기서 네비게이션 노드에 목적지를 전달하지만,
        # 테스트용으로 모터를 살짝 앞으로 굴리는 명령을 보내볼 수 있습니다.
        move_msg = Twist()
        move_msg.linear.x = 0.2 # 0.2m/s 속도로 전진
        self.vel_pub.publish(move_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()