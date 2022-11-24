
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
                                   'config', 'map1.yaml'),
        )

    map_server = Node(
        parameters=[
          {'yaml_filename': 'map1.yaml'},
        ],
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        )
        
    amcl_node=  Node(
        parameters=[
            '--use_map_topic'
        ],
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen')

    ld.add_action(declare_path_map)
    ld.add_action(map_server)
    ld.add_action(amcl_node)

    return ld


