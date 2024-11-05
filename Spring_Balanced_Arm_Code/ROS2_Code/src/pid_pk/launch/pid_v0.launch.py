from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(
            package= 'pid_pk',
            executable='pid_controller_spring_node',
            output='screen'
        ),
    ])