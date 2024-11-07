#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    
    ROBOT_NAMESPACE = os.environ['ROBOT_NAMESPACE']
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_namespace = LaunchConfiguration('use_namespace', default='false')

    # Archivos URDF dependiendo de si se usa namespace
    urdf_file_name_ns = 'turtlebot3_' + TURTLEBOT3_MODEL + '_' + ROBOT_NAMESPACE + '.urdf'
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'

    # Define las rutas a los fichero URDF
    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        urdf_file_name)

    urdf_ns = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        urdf_file_name_ns)

    # Escogemos un URDF según el valor de use_namespace
    urdf_to_use = PythonExpression([
        "'",
        urdf_ns,
        "' if '",
        use_namespace,
        "' == 'True' else '",
        urdf,
        "'"
    ])

    #Leemos el fichero urdf y lo cargamos en la variable robot_description_param
    robot_description_param = PythonExpression(["open('", urdf_to_use, "', 'r').read()"])

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_namespace',
            default_value='false',
            description='Indica si se debe usar un namespace'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Usar el reloj de simulación (Gazebo) si es true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_param,
                'use_sim_time': use_sim_time
            }]
        )
    ])
