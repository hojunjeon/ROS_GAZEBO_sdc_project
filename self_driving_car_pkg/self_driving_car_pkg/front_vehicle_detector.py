import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np

class FrontVehicleDetector(Node):
    def __init__(self):
        super().__init__('front_vehicle_detector')
        self.lidar_sub = self.create_subscription(
            LaserScan, '/lidar/lidar_scan', self.lidar_callback, 10)
        self.distance_pub = self.create_publisher(
            Float32, '/front_vehicle_distance', 10)
        self.prev_front_distance = None
        # 유예(Grace Period) 관련 변수
        self.lost_count = 0
        self.lost_threshold = 4  # 4프레임(2초) 동안 마지막 거리값 유지
        self.last_valid_distance = None

    def lidar_callback(self, msg):
        angle_range = 30  # degree
        center_idx = int((0 - msg.angle_min) / msg.angle_increment)
        idx_range = int(angle_range * np.pi / 180 / msg.angle_increment)
        start = max(center_idx - idx_range, 0)
        end = min(center_idx + idx_range, len(msg.ranges) - 1)
        valid_ranges = [r for r in msg.ranges[start:end+1] if msg.range_min < r < msg.range_max]
        msg_out = Float32()
        if valid_ranges:
            front_distance = min(valid_ranges)
            self.last_valid_distance = front_distance
            self.lost_count = 0
            msg_out.data = float(front_distance)
        else:
            # 차량 미감지: 유예 프레임 동안 마지막 거리값 publish, 이후 -1.0
            if self.last_valid_distance is not None and self.lost_count < self.lost_threshold:
                msg_out.data = float(self.last_valid_distance)
                self.lost_count += 1
            else:
                msg_out.data = -1.0
        self.distance_pub.publish(msg_out)
        self.prev_front_distance = msg_out.data

def main(args=None):
    rclpy.init(args=args)
    node = FrontVehicleDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
