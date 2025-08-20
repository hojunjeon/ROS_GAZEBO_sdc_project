import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np

class MaintainDistanceNode(Node):
    def __init__(self):
        super().__init__('maintain_distance_node')
        self.lidar_sub = self.create_subscription(
            LaserScan, '/lidar/lidar_scan', self.lidar_callback, 10)
        self.distance_pub = self.create_publisher(Float32, '/front_vehicle_distance', 10)
        self.front_distance = None

    def lidar_callback(self, msg: LaserScan):
        # 전방 10도 범위 평균 거리 계산
        angle_range = 20  # degree
        center_idx = int((0 - msg.angle_min) / msg.angle_increment)
        idx_range = int(angle_range * np.pi / 180 / msg.angle_increment)
        start = max(center_idx - idx_range, 0)
        end = min(center_idx + idx_range, len(msg.ranges) - 1)
        valid_ranges = [r for r in msg.ranges[start:end+1] if msg.range_min < r < msg.range_max]
        if valid_ranges:
            self.front_distance = min(valid_ranges)
        else:
            self.front_distance = -1.0
        # 거리값 publish
        msg_out = Float32()
        msg_out.data = float(self.front_distance)
        self.distance_pub.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = MaintainDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()