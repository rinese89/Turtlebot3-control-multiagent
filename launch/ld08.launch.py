#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    
    ROBOT_NAMESPACE = os.environ['ROBOT_NAMESPACE']

    #if (ROBOT_NAMESPACE==''):
    #frame_id='frame_id:=base_scan'
    #else:
    #frame_id= 'frame_id:='+ROBOT_NAMESPACE +'/base_scan'
    frame_id= 'frame_id:=tb3_0/base_scan'

    return LaunchDescription([
        Node(
            package='ld08_driver',
            executable='ld08_driver',
            name='ld08_driver',
            arguments=['--ros-args', '--param', frame_id],
            output='screen',
            #remappings=[
            #    ('/tf', 'tf'),
            #    ('/tf_static', 'tf_static')],
            ),
            
    ])
