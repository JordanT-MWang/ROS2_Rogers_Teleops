from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(
            package= 'pid_pkg',
            executable='pd_controller_node_gravity_balanced',
            output='screen'
        ),
    ])