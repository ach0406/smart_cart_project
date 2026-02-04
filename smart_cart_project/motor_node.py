import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # 핀 설정 (하드웨어 설계도 기반)
        self.PWMA, self.AIN1, self.AIN2 = 12, 17, 27
        self.PWMB, self.BIN1, self.BIN2 = 13, 22, 23
        
        GPIO.setmode(GPIO.BCM)
        for pin in [self.PWMA, self.AIN1, self.AIN2, self.PWMB, self.BIN1, self.BIN2]:
            GPIO.setup(pin, GPIO.OUT)
        
        self.pwm_a = GPIO.PWM(self.PWMA, 1000)
        self.pwm_b = GPIO.PWM(self.PWMB, 1000)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

    def cmd_vel_callback(self, msg):
        # 단순화된 구동 로직 (전진/후진)
        speed = min(abs(msg.linear.x) * 100, 100)
        if msg.linear.x > 0: # 전진
            GPIO.output(self.AIN1, True); GPIO.output(self.AIN2, False)
        else: # 후진
            GPIO.output(self.AIN1, False); GPIO.output(self.AIN2, True)
        self.pwm_a.ChangeDutyCycle(speed)
        self.pwm_b.ChangeDutyCycle(speed)

def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    rclpy.spin(node)
    GPIO.cleanup()
    node.destroy_node()
    rclpy.shutdown()