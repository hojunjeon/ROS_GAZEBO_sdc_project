#랜덤 시점 3초 정지 / 60,90 중 랜덤 속도로 등속직진
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import random
import cv2
from numpy import interp
from .Drive_Bot import Car, Debugging

class LaneAndSpeedPrius(Node):
    def __init__(self):
        super().__init__('lane_and_speed_prius')
        self.publisher = self.create_publisher(Twist, '/prius_hybrid/cmd_vel', 10)
        self.subscriber = self.create_subscription(
            Image, '/prius_hybrid/camera/image_raw', self.process_image, 10)
        self.bridge = CvBridge()
        self.Car = Car()
        self.Debug = Debugging()
        self.velocity = Twist()
        # 속도 제어 상태
        self.speed_choices = [60.0, 90.0]
        self.current_speed = random.choice(self.speed_choices)
        self.state = 'run'  # 'run' 또는 'stop'
        self.last_state_change = self.get_clock().now()
        self.stop_duration = 3.0  # 정차 시간(초)
        self.next_stop_time = self.get_clock().now() + rclpy.duration.Duration(seconds=random.uniform(8, 20))

    def process_image(self, data):
        self.Debug.setDebugParameters()
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        angle, lane_speed, img = self.Car.driveCar(frame, debug_print=False)

        now = self.get_clock().now()
        # 코너에서는 항상 30 유지
        if abs(angle) >= 0.1:
            speed = 30.0
            self.current_speed = 30.0
            self.state = 'run'
            self.last_state_change = now
            self.next_stop_time = now + rclpy.duration.Duration(seconds=random.uniform(8, 20))
        else:
            # 직선 구간에서만 정차/주행 반복
            if self.state == 'run':
                if now >= self.next_stop_time:
                    self.state = 'stop'
                    self.current_speed = 0.0
                    self.last_state_change = now
                else:
                    # 주행 중에는 고정된 속도 유지
                    pass
            elif self.state == 'stop':
                if (now - self.last_state_change).nanoseconds / 1e9 >= self.stop_duration:
                    self.state = 'run'
                    # 정지 후 주행 재개 시 속도를 30, 60, 90 중 랜덤 선택
                    self.current_speed = random.choice(self.speed_choices)
                    self.last_state_change = now
                    self.next_stop_time = now + rclpy.duration.Duration(seconds=random.uniform(8, 20))
                else:
                    self.current_speed = 0.0
            speed = self.current_speed

        print(f"[prius_hybrid] : 속도 = {speed:.2f} / 조향각 = {angle:.2f}")

        if speed != 0:
            speed = interp(speed, [20, 90], [1, 4])
        self.velocity.linear.x = speed
        self.velocity.angular.z = angle        
        self.publisher.publish(self.velocity)

        # 디버깅용 시각화
        cv2.imshow("Lead Car Lane Following", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LaneAndSpeedPrius()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()