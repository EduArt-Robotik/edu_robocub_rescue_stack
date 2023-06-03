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
from nav2_common.launch import RewrittenYaml





def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('edu_robocup_rescue_stack')
    slamtoolbox_dir = get_package_share_directory('slam_toolbox')
    #slamtoolbox_launch_dir = os.path.join(slamtoolbox_dir, 'launch')
    namespace = LaunchConfiguration('namespace')
    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
    nav2_launch_dir = os.path.join(nav2_dir, 'launch') 
    namespace = LaunchConfiguration('namespace')
    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    mask_yaml_file = LaunchConfiguration('mask')


    # lifecycle nodes
    lifecycle_nodes_localisation = ['amcl', 'map_server']
    lifecycle_nodes_costmap_filters = ['costmap_filter_info_server', 'filter_mask_server']   

    # remapping
    remap_odom = LaunchConfiguration('remap_odom', default='/odometry/filtered')

    declare_namespace = DeclareLaunchArgument(
        name = 'namespace',
        default_value = ''
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_remap_odom = DeclareLaunchArgument(
        'remap_odom',
        default_value='/odometry/filtered',
        description='/odom'
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
                                   'map', 'map_9.1.yaml')
    )

    declare_map_subscribe_transient_local = DeclareLaunchArgument(
            'map_subscribe_transient_local', default_value='true',
            description='Whether to set the map subscriber QoS to transient local'
    )

    declare_mask_yaml_file = DeclareLaunchArgument(
        'mask',
        default_value=os.path.join(bringup_dir, 'keepout', 'keepout_8.2.yaml')
    )

    start_lifecycle_manager_localisation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes_localisation}]
    )

    # Nodes launching commands
    start_lifecycle_manager_costmap_filters = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            namespace=namespace,
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes_costmap_filters}])

    start_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}, {'use_sim_time': use_sim_time}]
    )

    start_bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    start_amcl = Node(
       package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/odom', remap_odom)]
    )



    start_edu_robocup_rescue_stack_node = Node (
        package='edu_robocup_rescue_stack', 
        executable='edu_robocup_rescue_stack_node',
        output='screen'
    )

    start_tf_map_base_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'map']
    )
        
    start_tf_base_link_laser_link = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0.0', '--y', '0', '--z', '0.139', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'laser_link']
    )

    start_tf_base_link_imu_link = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments= ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'imu_link']
    )   # arguments need to be checked !!!!

    

    start_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')), 
        launch_arguments = {'namespace':namespace,
                           'use_sim_time': use_sim_time,
                            'map': map_file,
                            'params_file': params_file,
                            'autostart': autostart}.items()
    )

    # filter mask
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': mask_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    start_costmap_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[configured_params])
    
    start_filter_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[configured_params]
    )

    # robot localisation node
    start_robot_localization_node = Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[os.path.join(bringup_dir, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}], #, {'use_sim_time': use_sim_time}]
         
    )

    ld = LaunchDescription()    
    #ld.add_action(start_slam_toolbox)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_remap_odom)
    ld.add_action(declare_namespace)
    ld.add_action(declare_auto_start)
    ld.add_action(declare_params_file)  
    ld.add_action(declare_map_file)
    ld.add_action(declare_map_subscribe_transient_local)
    ld.add_action(declare_mask_yaml_file)
    ld.add_action(start_map_server)
    #ld.add_action(start_amcl)
    
    
    ld.add_action(start_edu_robocup_rescue_stack_node)
    #ld.add_action(start_tf_map_base_frame)
    ld.add_action(start_tf_base_link_laser_link)
    ld.add_action(start_tf_base_link_imu_link)
    ld.add_action(start_navigation)
    ld.add_action(start_bt_navigator)
    #ld.add_action(start_robot_localization_node)
    ld.add_action(start_costmap_filter_info_server)
    ld.add_action(start_filter_mask_server)
    ld.add_action(start_lifecycle_manager_localisation)
    ld.add_action(start_lifecycle_manager_costmap_filters)
    

    return ld