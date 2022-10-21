import os
from launch import LaunchDescription
from launch_ros.actions import Node
 
 
def generate_launch_description():
 
  return LaunchDescription([
    Node(package='edu_robocup_rescue_stack', executable='edu_robocup_rescue_stack_node.cpp',
      output='screen'),
  ])
