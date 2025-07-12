import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from scripts import GazeboRosPaths


def generate_launch_description():
    package_share_dir = get_package_share_directory("arm_detecting_controll")
    urdf_file = os.path.join(package_share_dir, "urdf", "arm_robot.urdf")
    world_file = os.path.join(package_share_dir, 'world', 'maze.world')
    controller_file = os.path.join(package_share_dir, "config", "rdc_arm.yaml")
    with open(urdf_file, 'r') as inf:
        robot_description_content = inf.read()

    robot_description = {"robot_description": robot_description_content}
    os.environ['GAZEBO_MODEL_PATH'] = os.path.join(package_share_dir, 'models')

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["gazebo","-s","libgazebo_ros_factory.so",world_file],
                output="screen",
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-entity","arm_detecting_controll", "-b", "-file", urdf_file,],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                arguments=[urdf_file],
            ),

            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                ),

            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["left_wheel_controller", "-c", "/controller_manager"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["right_wheel_controller", "-c", "/controller_manager"],
            ),
        ]
    )
