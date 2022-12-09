import os
#import argparse
#import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ros2run.api import get_executable_path
#from nav2_common.launch import RewrittenYaml
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('edu_robocup_rescue_stack')
    slamtoolbox_dir = get_package_share_directory('slam_toolbox')
    slamtoolbox_launch_dir = os.path.join(slamtoolbox_dir, 'launch')
    namespace = LaunchConfiguration('namespace')
    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
    nav2_launch_dir = os.path.join(nav2_dir, 'launch') 
    namespace = LaunchConfiguration('namespace')
    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    lifecycle_nodes = ['map_server', 'amcl']

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_file}


    
        # Set env var to print messages to stdout immediately
        #SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),



    declare_namespace = DeclareLaunchArgument(
        name = 'namespace',
        default_value = ''
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_auto_start =    DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_edu_params.yaml'),
        description='Full path to the ROS2 parameters file to use'
    )

        
    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(get_package_share_directory('edu_robocup_rescue_stack'),
                                   'map', 'map4.yaml')
    )

    declare_map_subscribe_transient_local = DeclareLaunchArgument(
            'map_subscribe_transient_local', default_value='true',
            description='Whether to set the map subscriber QoS to transient local'
    )

    start_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file},],
    )

    start_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file],
    )

    start_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes}]
    )

    start_edu_robocup_rescue_stack_node = Node (
        package='edu_robocup_rescue_stack', 
        executable='edu_robocup_rescue_stack_node',
         output='screen'
    )

    start_tf_map_base_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'map', '--child-frame-id', 'base_frame']
    )
        
    start_tf_base_frame_laser_link = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0.1', '--y', '0', '--z', '0.16', '--yaw', '-0.4', '--pitch', '0', '--roll', '0', '--frame-id', 'base_frame', '--child-frame-id', 'laser_link']
    )


    start_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')), 
        launch_arguments = {'namespace':namespace,
                            'use_sim_time': use_sim_time,
                            'map': map_file,
                            'params_file': params_file,
                            'autostart': autostart}.items()
    )

    #start_slam_toolbox = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(os.path.join(slamtoolbox_launch_dir, 'online_async_launch.py'))
    #)

    ld = LaunchDescription()    
    #ld.add_action(start_slam_toolbox)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_namespace)
    ld.add_action(declare_auto_start)
    ld.add_action(declare_params_file)  
    ld.add_action(declare_map_file)
    ld.add_action(declare_map_subscribe_transient_local)
    ld.add_action(start_map_server)
    ld.add_action(start_amcl)
    ld.add_action(start_lifecycle_manager)
    ld.add_action(start_edu_robocup_rescue_stack_node)
    ld.add_action(start_tf_map_base_frame)
    ld.add_action(start_tf_base_frame_laser_link)
    ld.add_action(start_navigation)
    

    return ld
