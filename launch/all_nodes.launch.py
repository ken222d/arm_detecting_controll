from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_detecting_controll',
            executable='linetrace_controller',
            name='linetrace_controller',
            output='screen'
        ),
        Node(
            package='arm_detecting_controll',
            executable='multi_color_ball_detector',
            name='multi_color_ball_detector',
            output='screen'
        ),
        Node(
            package='arm_detecting_controll',
            executable='arm_controll',
            name='arm_controll',
            output='screen'
        ),
        Node(
            package='arm_detecting_controll',
            executable='ball_follower',
            name='ball_follower',
            output='screen'
        ),
    ])

