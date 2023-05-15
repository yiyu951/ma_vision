import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    camera_node = ComposableNode(
        package="mindvision_ros",
        plugin="mindvision_camera::MVCameraNode",
        name="camera",
    )

    detect_node = ComposableNode(package="auto_aim",
                                 plugin="MA::AutoAimNode",
                                 name="auto_aim")


    container = ComposableNodeContainer(
        name="my_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="both",
        composable_node_descriptions=[camera_node, detect_node],
        # arguments=['--ros-args', '--log-level', 'debug'],
    )

    ld = LaunchDescription()

    ld.add_action(container)

    return ld
