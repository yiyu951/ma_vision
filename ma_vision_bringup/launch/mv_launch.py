import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # params_file = os.path.join(
        # get_package_share_directory('mindvision_camera'), 'config', 'camera_params.yaml')

    # camera_info_url = 'package://mindvision_camera/config/camera_info.yaml'

    return LaunchDescription([

        Node(
            package='mindvision_ros',
            executable='mindvision_ros_node',
            output='screen',
            emulate_tty=True,
            # parameters=[],
        )
    ])
