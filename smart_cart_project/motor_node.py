import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # 핀 설정 (메카넘 4륜 기준 - BCM 번호)
        # 드라이버 1 (앞바퀴)
        self.FL_PWM, self.FL_IN1, self.FL_IN2 = 12, 17, 27
        self.FR_PWM, self.FR_IN1, self.FR_IN2 = 13, 22, 23
        # 드라이버 2 (뒷바퀴)
        self.RL_PWM, self.RL_IN1, self.RL_IN2 = 18, 24, 25
        self.RR_PWM, self.RR_IN1, self.RR_IN2 = 19, 5, 6
        
        GPIO.setmode(GPIO.BCM)
        pins = [self.FL_PWM, self.FL_IN1, self.FL_IN2, self.FR_PWM, self.FR_IN1, self.FR_IN2,
                self.RL_PWM, self.RL_IN1, self.RL_IN2, self.RR_PWM, self.RR_IN1, self.RR_IN2]
        
        for pin in pins:
            GPIO.setup(pin, GPIO.OUT)
        
        # PWM 시작
        self.pwm_fl = GPIO.PWM(self.FL_PWM, 1000); self.pwm_fl.start(0)
        self.pwm_fr = GPIO.PWM(self.FR_PWM, 1000); self.pwm_fr.start(0)
        self.pwm_rl = GPIO.PWM(self.RL_PWM, 1000); self.pwm_rl.start(0)
        self.pwm_rr = GPIO.PWM(self.RR_PWM, 1000); self.pwm_rr.start(0)
        
        self.get_logger().info('메카넘 휠 모터 노드가 준비되었습니다.')

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x    # 전진/후진
        vy = msg.linear.y    # 좌우 이동
        wz = msg.angular.z   # 회전

        # 메카넘 휠 역기하학 계산
        speeds = [
            vx - vy - wz,  # Front Left
            vx + vy + wz,  # Front Right
            vx + vy - wz,  # Rear Left
            vx - vy + wz   # Rear Right
        ]

        motors = [
            (self.pwm_fl, self.FL_IN1, self.FL_IN2),
            (self.pwm_fr, self.FR_IN1, self.FR_IN2),
            (self.pwm_rl, self.RL_IN1, self.RL_IN2),
            (self.pwm_rr, self.RR_IN1, self.RR_IN2)
        ]

        for i, speed in enumerate(speeds):
            pwm, in1, in2 = motors[i]
            if speed > 0:
                GPIO.output(in1, True); GPIO.output(in2, False)
            elif speed < 0:
                GPIO.output(in1, False); GPIO.output(in2, True)
            else:
                GPIO.output(in1, False); GPIO.output(in2, False)
            
            duty = min(abs(speed) * 100, 100)
            pwm.ChangeDutyCycle(duty)

def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()