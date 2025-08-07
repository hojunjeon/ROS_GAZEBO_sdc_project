from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('self_driving_car_pkg')
    world = os.path.join(pkg, 'worlds', 'detect_test.world')
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
    ])

