import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import time

class UltrasonicSensor(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.publisher_ = self.create_publisher(Range, '/ultrasonic_range', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz
        
        # 핀 설정 (내일 조립 시 이 번호에 맞게 꽂으세요)
        self.TRIG = 24
        self.ECHO = 25
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)

    def timer_callback(self):
        GPIO.output(self.TRIG, True)
        time.sleep(0.00001)
        GPIO.output(self.TRIG, False)

        while GPIO.input(self.ECHO) == 0:
            start_time = time.time()
        while GPIO.input(self.ECHO) == 1:
            stop_time = time.time()

        duration = stop_time - start_time
        distance = (duration * 34300) / 2 # cm 단위 계산

        msg = Range()
        msg.range = float(distance)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Distance: {distance:.2f} cm')

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        GPIO.cleanup()
    node.destroy_node()
    rclpy.shutdown()