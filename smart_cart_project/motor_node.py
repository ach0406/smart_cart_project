import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # [Step 4] 물리 핀 -> BCM 번호 매핑 완료
        self.STBY_PIN = 25  # 물리 22번

        # 드라이버 1 (앞바퀴) - 물리 12, 16, 18 / 33, 13, 15
        self.FL_PWM, self.FL_IN1, self.FL_IN2 = 18, 23, 24
        self.FR_PWM, self.FR_IN1, self.FR_IN2 = 13, 27, 22
        
        # 드라이버 2 (뒷바퀴) - 물리 32, 36, 38 / 35, 37, 40
        self.RL_PWM, self.RL_IN1, self.RL_IN2 = 12, 16, 20
        self.RR_PWM, self.RR_IN1, self.RR_IN2 = 19, 26, 21

        GPIO.setmode(GPIO.BCM)
        pins = [self.STBY_PIN, self.FL_PWM, self.FL_IN1, self.FL_IN2, self.FR_PWM, self.FR_IN1, self.FR_IN2,
                self.RL_PWM, self.RL_IN1, self.RL_IN2, self.RR_PWM, self.RR_IN1, self.RR_IN2]
        for pin in pins: GPIO.setup(pin, GPIO.OUT)
        
        GPIO.output(self.STBY_PIN, True) # 모터 드라이버 활성화
        
        self.pwm_fl = GPIO.PWM(self.FL_PWM, 1000); self.pwm_fl.start(0)
        self.pwm_fr = GPIO.PWM(self.FR_PWM, 1000); self.pwm_fr.start(0)
        self.pwm_rl = GPIO.PWM(self.RL_PWM, 1000); self.pwm_rl.start(0)
        self.pwm_rr = GPIO.PWM(self.RR_PWM, 1000); self.pwm_rr.start(0)

        self.get_logger().info('✅ 모터 자가 진단 시작!')
        self.run_self_test()

    def run_self_test(self):
        motors = [(self.pwm_fl, self.FL_IN1, self.FL_IN2), (self.pwm_fr, self.FR_IN1, self.FR_IN2),
                  (self.pwm_rl, self.RL_IN1, self.RL_IN2), (self.pwm_rr, self.RR_IN1, self.RR_IN2)]
        for pwm, in1, in2 in motors:
            GPIO.output(in1, True); pwm.ChangeDutyCycle(30); time.sleep(0.5)
            pwm.ChangeDutyCycle(0); GPIO.output(in1, False); time.sleep(0.2)
        self.get_logger().info('✨ 자가 진단 완료.')

    def cmd_vel_callback(self, msg):
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z
        speeds = [vx-vy-wz, vx+vy+wz, vx+vy-wz, vx-vy+wz]
        motors = [(self.pwm_fl, self.FL_IN1, self.FL_IN2), (self.pwm_fr, self.FR_IN1, self.FR_IN2),
                  (self.pwm_rl, self.RL_IN1, self.RL_IN2), (self.pwm_rr, self.RR_IN1, self.RR_IN2)]
        for i, speed in enumerate(speeds):
            pwm, in1, in2 = motors[i]
            GPIO.output(in1, speed > 0); GPIO.output(in2, speed < 0)
            pwm.ChangeDutyCycle(min(abs(speed) * 100, 100))

def main(args=None):
    rclpy.init(args=args); node = MotorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: GPIO.cleanup(); node.destroy_node(); rclpy.shutdown()