
import argparse
import os
import sys

from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ros2run.api import get_executable_path
from launch_ros.actions import Node



def generate_launch_description():
    map_file = LaunchConfiguration('map_file')
 
    ld = LaunchDescription()

    declare_path_map = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(get_package_share_directory('edu_robocup_rescue_stack'),
                                   'config', 'map2.yaml'),
        )

    map_server = Node(
        parameters=[
          {'yaml_filename': map_file},
        ],
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        )
        
    amcl_node=  Node(
        parameters=[
           { 'scan_topic': '/demo/laser/out'},
           { 'base_frame_id': 'base_frame'},
        ],
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen')

    static_transform_publisher_map_odom_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'map', '--child-frame-id', 'odom']
        )
    static_transform_publisher_odom_base_frame_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'odom', '--child-frame-id', 'base_frame']
        )
        
    static_transform_publisher_base_frame_laser_link_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_frame', '--child-frame-id', 'laser_link']
        )

    
    start_localisation_control_node = Node (
        package='edu_robocup_rescue_stack', 
        executable='edu_robocup_rescue_stack_node',
         output='screen'
         )

    lifecycle_manager_node = Node (
        parameters=[
           { 'autostart': True},
           { 'node_names': ['amcl', 'map_server']},
        ],
        package='nav2_lifecycle_manager', 
        executable='lifecycle_manager',
        output='screen'
        )
    

    ld.add_action(declare_path_map)
    ld.add_action(map_server)
    ld.add_action(amcl_node)
    ld.add_action(static_transform_publisher_map_odom_node)
    ld.add_action(static_transform_publisher_odom_base_frame_node)
    ld.add_action(static_transform_publisher_base_frame_laser_link_node)
    ld.add_action(lifecycle_manager_node)
    ld.add_action(start_localisation_control_node)


    return ld


