import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # load crazyflies
    crazyflies_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'crazyflies.yaml')

    with open(crazyflies_yaml, 'r') as ymlfile:
        crazyflies = yaml.safe_load(ymlfile)

    # load crazyflie_types
    crazyflies_types_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'crazyflie_types.yaml')

    with open(crazyflies_types_yaml, 'r') as ymlfile:
        crazyflie_types = yaml.safe_load(ymlfile)

    # construct motion_capture_configuration
    motion_capture_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'motion_capture.yaml')

    with open(motion_capture_yaml, 'r') as ymlfile:
        motion_capture = yaml.safe_load(ymlfile)

    motion_capture_params = motion_capture["/motion_capture_tracking"]["ros__parameters"]
    motion_capture_params["rigid_bodies"] = dict()
    for key, value in crazyflies.items():
        type = crazyflie_types[value["type"]]
        
        motion_capture_params["rigid_bodies"][key] =  {
                "initial_position": value["initial_position"],
                "marker": type["marker"],
                "dynamics": type["dynamics"],
            }

    # construct crazyflie_server configuration
    server_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'crazyflie_server.yaml')
    
    with open(server_yaml, 'r') as ymlfile:
        server_params = yaml.safe_load(ymlfile)

    server_params = server_params["/crazyflie_server"]["ros__parameters"]
    server_params["crazyflies"] = crazyflies
    server_params["crazyflie_types"] = crazyflie_types

    # teleop params
    teleop_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'teleop.yaml')
    teleop_5_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'teleop_5.yaml')
    
    return LaunchDescription([
        Node(
            package='motion_capture_tracking',
            executable='motion_capture_tracking_node',
            name='motion_capture_tracking',
            output='screen',
            parameters=[motion_capture_params]
        ),
        Node(
            package='crazyflie',
            executable='teleop',
            name='teleop',
            remappings=[
                ('takeoff', 'cf231/takeoff'),
                ('land', 'cf231/land'),
                ('cmd_vel', 'cf231/cmd_vel'),
                ('cmd_full_state', 'cf231/cmd_full_state'),
                ('notify_setpoints_stop', 'cf231/notify_setpoints_stop'),
                ('joy', 'cf231/joy'),
            ],
            parameters=[teleop_yaml]
        ),
        Node(
            package='crazyflie',
            executable='teleop',
            name='teleop',
            remappings=[
                ('takeoff', 'cf5/takeoff'),
                ('land', 'cf5/land'),
                ('cmd_vel', 'cf5/cmd_vel'),
                ('cmd_full_state', 'cf5/cmd_full_state'),
                ('notify_setpoints_stop', 'cf5/notify_setpoints_stop'),
                ('joy', 'cf5/joy'),
            ],
            parameters=[teleop_5_yaml]
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            remappings=[('joy', 'cf231/joy')],
            output='screen',
            parameters=[{'device_id':0}] # new joystick
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            remappings=[('joy', 'cf5/joy')],
            parameters=[{'device_id':1}] # old
        ),  
        Node(
            package='crazyflie',
            executable='crazyflie_server',
            name='crazyflie_server',
            output='screen',
            parameters=[server_params]
        ),
    ])

