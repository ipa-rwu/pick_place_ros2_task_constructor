import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    mock_io_server = Node(
        package="pick_place_app",
        executable="mock_io_server",
        output="screen",
    )

    return LaunchDescription([mock_io_server])
