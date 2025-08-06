import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist
import math
import time

class PedestrianController(Node):
    def __init__(self):
        super().__init__('pedestrian_controller_node')
        self.cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 대기 중: /gazebo/set_entity_state ...')

        self.pedestrian_names = ['pedestrian_1', 'pedestrian_2', 'pedestrian_3']
        self.crosswalk_y_positions = [20.0, 40.0, 70.0]  # 각 보행자 y 위치 (횡단보도)

        self.timer = self.create_timer(0.1, self.update_pedestrians)  # 10Hz
        self.start_time = self.get_clock().now()
        self.direction = {name: 1 for name in self.pedestrian_names}  # +1: 앞으로, -1: 뒤로

    def update_pedestrians(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9  # 초 단위로 변환
        period = 20.0  # 한 주기: 5초 이동 + 5초 대기 + 5초 이동 + 5초 대기

        for i, name in enumerate(self.pedestrian_names):
            local_time = elapsed % period

            # 보행자 위치 계산
            if 0 <= local_time < 5:  # -5 → +5 이동
                x = -5 + (local_time / 5.0) * 10
            elif 5 <= local_time < 10:  # 정지
                x = 5
            elif 10 <= local_time < 15:  # +5 → -5 이동
                x = 5 - ((local_time - 10) / 5.0) * 10
            else:  # 정지
                x = -5

            y = self.crosswalk_y_positions[i]
            self.set_pedestrian_position(name, x, y)

    def set_pedestrian_position(self, model_name, x, y):
        req = SetEntityState.Request()
        state = EntityState()
        state.name = model_name

        # 위치 설정
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = 0.0
        state.pose.orientation.x = 0.0
        state.pose.orientation.y = 0.0
        state.pose.orientation.z = 0.0
        state.pose.orientation.w = 1.0

        # 속도는 0
        state.twist = Twist()

        req.state = state

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.1)
        if future.done() and future.result() is not None:
            pass
        else:
            self.get_logger().warn(f"{model_name} 위치 설정 실패")

def main(args=None):
    rclpy.init(args=args)
    node = PedestrianController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
