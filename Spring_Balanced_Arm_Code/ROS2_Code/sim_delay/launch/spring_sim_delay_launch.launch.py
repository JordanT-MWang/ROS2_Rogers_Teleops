from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(
            package= 'sim_delay',
            executable='spring_sim_delay_node',
            output='screen'
        ),
    ])