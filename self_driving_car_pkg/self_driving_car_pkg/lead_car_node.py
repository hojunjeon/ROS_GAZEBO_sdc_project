
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import random
import cv2
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
        self.current_speed = 0.0
        self.target_speed = 0.0
        self.last_change_time = self.get_clock().now()
        self.hold_time = 3.0
        self.accel_time = 1.0
        self.state = 'accel'
        self.last_update_time = self.get_clock().now()

    def process_image(self, data):
        self.Debug.setDebugParameters()
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        angle, lane_speed, img = self.Car.driveCar(frame)

        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now

        # 직선/코너 판별 (조향각 임계값)
        if abs(angle) < 0.1:
            # 직선: 랜덤 속도 알고리즘 적용
            if self.state == 'accel':
                if abs(self.current_speed - self.target_speed) < 0.5:
                    self.current_speed = self.target_speed
                    self.state = 'hold'
                    self.last_change_time = now
                else:
                    speed_diff = self.target_speed - self.current_speed
                    step = speed_diff * min(dt / self.accel_time, 1.0)
                    self.current_speed += step
            elif self.state == 'hold':
                if (now - self.last_change_time).nanoseconds / 1e9 >= self.hold_time:
                    self.target_speed = random.uniform(1.0, 5.0)
                    self.state = 'accel'
                    self.last_change_time = now
            # 최초 진입 시 타겟 속도 설정
            if self.target_speed == 0.0:
                self.target_speed = random.uniform(1.0, 5.0)
            speed = self.current_speed
        else:
            # 코너: 차선 인식 속도 사용, 상태 초기화
            speed = lane_speed
            self.current_speed = lane_speed
            self.target_speed = 0.0
            self.state = 'accel'

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