
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

    map1_file = LaunchConfiguration('map1_file')
    map2_file = LaunchConfiguration('map2_file')
    map3_file = LaunchConfiguration('map3_file')

    ld = LaunchDescription()

    declare_path_map1 = DeclareLaunchArgument(
        'map1_file',
        default_value=os.path.join(get_package_share_directory('edu_robocup_rescue_stack'), 'config', 'map_flat_start.yaml'),
        )

    declare_path_map2 = DeclareLaunchArgument(
        'map2_file',
        default_value=os.path.join(get_package_share_directory('edu_robocup_rescue_stack'), 'config', 'map_ramp_start.yaml'),
        )
    
    declare_path_map3 = DeclareLaunchArgument(
        'map3_file',
        default_value=os.path.join(get_package_share_directory('edu_robocup_rescue_stack'), 'config', 'map_ramp_end.yaml'),
        )


    map1_server = Node(
        parameters=[
          {'yaml_filename': map1_file},
        ],
        package='nav2_map_server',
        executable='map_server',
        name='map1_server',
        )

    map2_server = Node(
        parameters=[
          {'yaml_filename': map2_file},
        ],
        package='nav2_map_server',
        executable='map_server',
        name='map2_server',
        )
    
    map3_server = Node(
        parameters=[
          {'yaml_filename': map3_file},
        ],
        package='nav2_map_server',
        executable='map_server',
        name='map3_server',
        )
        
    amcl_node =  Node(
        parameters=[
           { 'scan_topic': '/demo/laser/out'},
           { 'base_frame_id': 'base_frame'},
        ],
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen')
        
    static_transform_publisher_base_frame_laser_link_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0.1', '--y', '0', '--z', '0.16', '--yaw', '-0.4', '--pitch', '0', '--roll', '0', '--frame-id', 'base_frame', '--child-frame-id', 'laser_link']        )

    
    start_localisation_control_node = Node (
        package='edu_robocup_rescue_stack', 
        executable='edu_robocup_rescue_stack_node',
         output='screen'
         )

    lifecycle_manager_node = Node (
        parameters=[
           { 'autostart': True},
           { 'node_names': ['amcl']},
        ],
        package='nav2_lifecycle_manager', 
        executable='lifecycle_manager',
        output='screen'
        )
    

    ld.add_action(declare_path_map1)
    ld.add_action(declare_path_map2)
    ld.add_action(declare_path_map3)
    ld.add_action(map1_server)
    ld.add_action(map2_server)
    ld.add_action(map3_server)

    ld.add_action(amcl_node)
    #ld.add_action(static_transform_publisher_map_odom_node)
    #ld.add_action(static_transform_publisher_odom_base_frame_node)
    ld.add_action(static_transform_publisher_base_frame_laser_link_node)
    ld.add_action(lifecycle_manager_node)
    ld.add_action(start_localisation_control_node)

    return ld


