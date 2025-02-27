import launch
import launch_ros
import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import yaml

import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory("crazyflie_server_py")

    crazyflie_node = launch_ros.actions.Node(
        package="crazyflie_server_py",
        executable="crazyflie_server",
        output="screen",
        emulate_tty=True,
    )

    ld = LaunchDescription()
    ld.add_action(crazyflie_node)

    return ld
