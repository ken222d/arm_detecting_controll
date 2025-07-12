from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ライントレースノード
        Node(
            package='arm_detecting_controll',
            executable='linetrace_controller',
            name='line_trace_controller',
            output='screen'
        ),

        # カメラノード（ボール検出）
        Node(
            package='arm_detecting_controll',
            executable='multi_color_ball_detector',
            name='multi_color_ball_detector',
            output='screen'
        ),

        # ボール追従ノード
        Node(
            package='arm_detecting_controll',
            executable='ball_follower',
            name='ball_follower',
            output='screen'
        ),

        # アーム制御ノード
        Node(
            package='arm_detecting_controll',
            executable='attach_arm_ball',
            name='vacuum_arm_controller',
            output='screen'
        ),
    ])

