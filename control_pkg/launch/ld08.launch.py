#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():

    use_namespace = LaunchConfiguration('use_namespace')
    
    ROBOT_NAMESPACE = os.environ['ROBOT_NAMESPACE']

    frame_id= 'frame_id:=base_scan'
    frame_id_ns= 'frame_id:='+ ROBOT_NAMESPACE +'/base_scan'

    base_scan_frame_id = PythonExpression([
        "'",
        frame_id_ns,
        "' if '",
        use_namespace,
        "' == 'True' else '",
        frame_id,
        "'"
    ])    

    print(base_scan_frame_id)
    #print(frame_id)

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_namespace',
            default_value='false',
            description='Indica si se debe usar un namespace'),

        Node(
            package='ld08_driver',
            executable='ld08_driver',
            name='ld08_driver',
            arguments=['--ros-args', '--param', base_scan_frame_id],
            output='screen',
            ),
        
        #Node(
        #    package='ld08_driver',
        #    executable='ld08_driver',
        #    name='ld08_driver',
        #    arguments=['--ros-args', '--param', frame_id],
        #    output='screen',
        #    ),
            
    ])
