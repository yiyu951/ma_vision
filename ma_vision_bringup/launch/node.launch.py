import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    default_params_file = os.path.join(
        get_package_share_directory('ma_vision_bringup'),
        'config/default.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(name='detect_color',
                              default_value='1', description='0-Red 1-Blue'),
        DeclareLaunchArgument(name='debug',
                              default_value='true'),
        DeclareLaunchArgument(name='params_file',
                              default_value=default_params_file),

        Node(
            package='mindvision_ros',
            executable='mindvision_ros_node',
            name='mindvision',
            output='log',
            # emulate_tty=True,
            # parameters=[LaunchConfiguration('params_file'), {
            #     'detect_color': LaunchConfiguration('detect_color'),
            # }],
        ),

        Node(
            package='auto_aim',
            executable='auto_aim_node',
            output='both',
            # emulate_tty=True,
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])