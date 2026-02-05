import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import mediapipe as mp

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        # 주행 상태를 제어하는 토픽 발행 (path_planner가 이 메시지를 듣게 됩니다)
        self.command_pub = self.create_publisher(String, '/cart_command', 10)
        
        # 카메라 설정
        self.cap = cv2.VideoCapture(0)
        
        # MediaPipe Hands 설정
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        # 0.1초마다 프레임을 처리하는 타이머 (10 FPS)
        self.timer = self.create_timer(0.1, self.process_gesture)
        self.get_logger().info('제스처 인식(주행 재개) 노드가 시작되었습니다.')

    def process_gesture(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # MediaPipe 처리를 위해 RGB로 변환
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # '손바닥 펴기(보자기)' 제스처 감지 시 RESUME 명령 전송
                if self.is_open_palm(hand_landmarks):
                    msg = String()
                    msg.data = "RESUME"
                    self.command_pub.publish(msg)
                    self.get_logger().info('✋ 제스처 감지: 주행 재개(RESUME) 명령 전송')
                
                # 필요 시 화면에 랜드마크 그리기 (디버깅용)
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

        # 라즈베리파이에서 모니터를 연결했을 때만 창을 띄웁니다.
        # cv2.imshow('Gesture Recognition', frame)
        # cv2.waitKey(1)

    def is_open_palm(self, landmarks):
        # 손가락 끝(8, 12, 16, 20)이 마디(6, 10, 14, 18)보다 위에 있는지 확인
        # Y축 값은 위로 갈수록 작아지므로 '<' 연산자를 사용합니다.
        finger_tips = [8, 12, 16, 20]
        finger_pips = [6, 10, 14, 18]
        
        is_open = True
        for tip, pip in zip(finger_tips, finger_pips):
            if landmarks.landmark[tip].y > landmarks.landmark[pip].y:
                is_open = False
                break
        return is_open

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()