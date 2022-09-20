import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    # load crazyflies
    crazyflies_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'crazyflies_slam_example.yaml')

    with open(crazyflies_yaml, 'r') as ymlfile:
        crazyflies = yaml.safe_load(ymlfile)

    server_params = crazyflies

    # construct motion_capture_configuration
    motion_capture_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'motion_capture.yaml')

    with open(motion_capture_yaml, 'r') as ymlfile:
        motion_capture = yaml.safe_load(ymlfile)

    motion_capture_params = motion_capture["/motion_capture_tracking"]["ros__parameters"]
    motion_capture_params["rigid_bodies"] = dict()
    for key, value in crazyflies["robots"].items():
        type = crazyflies["robot_types"][value["type"]]
        if value["enabled"] and type["motion_capture"]["enabled"]:
            motion_capture_params["rigid_bodies"][key] =  {
                    "initial_position": value["initial_position"],
                    "marker": type["motion_capture"]["marker"],
                    "dynamics": type["motion_capture"]["dynamics"],
                }

    # teleop params
    teleop_params = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'teleop.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='cpp'),
        Node(
            package='crazyflie',
            executable='crazyflie_server.py',
            condition=LaunchConfigurationEquals('backend','cflib'),
            name='crazyflie_server',
            output='screen',
            parameters=[server_params]
        ),
        Node(
            package='crazyflie',
            executable='crazyflie_server',
            condition=LaunchConfigurationEquals('backend','cpp'),
            name='crazyflie_server',
            output='screen',
            parameters=[server_params]
        ),
        Node(
        parameters=[
          {'odom_frame': 'odom'},
          {'map_frame': 'world'},
          {'base_frame': 'cf2'},
          {'scan_topic': '/cf2/scan'},
          {'use_scan_matching': False},
          {'max_laser_range': 3.5},
          {'resolution': 0.1},
          {'minimum_travel_distance': 0.0},
          {'minimum_travel_heading': 0.0},
          {'map_update_interval': 0.1}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
    ])
