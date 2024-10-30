#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    
    #Declaracion de variables tomadas de las variables de entorno:  
    #ROBOT_NAMESPACE = os.environ['ROBOT_NAMESPACE']
    
    #Asignacion de los parametros de lazamiento a unas instancias que utilizaremos despues en el código
    namespace =LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_lsm = LaunchConfiguration('use_lsm')
    use_controller = LaunchConfiguration('use_controller')
    use_amcl = LaunchConfiguration('use_amcl')
    use_aruco = LaunchConfiguration('use_aruco')
    use_fusion_amcl_odom = LaunchConfiguration('use_fusion_amcl_odom')
    use_fusion_aruco_odom = LaunchConfiguration('use_fusion_aruco_odom')
    use_broadcast_ref = LaunchConfiguration('use_broadcast_ref')
    use_phi = LaunchConfiguration('use_phi')
    use_close_loop = LaunchConfiguration('use_close_loop')
    use_sim_time = LaunchConfiguration('use_sim_time')
    controller_type = LaunchConfiguration('controller_type')

    velocity = LaunchConfiguration('velocity')
    Ax = LaunchConfiguration('Ax')
    Ay = LaunchConfiguration('Ay')
    distance = LaunchConfiguration('distance')
    phi0 = LaunchConfiguration('phi0')
    period = LaunchConfiguration('period')

    #Rutas a los directorios de los paquetes
    localization_pkg_dir = get_package_share_directory('nav2_bringup')

    
    # Instancias con la ruta a otros launch
    localization_launch_file = PathJoinSubstitution(
        [localization_pkg_dir, 'launch', 'bringup_launch.py']) #launch del amcl
       
    # Declarar argumentos de launch
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='tb3_0',
        description='Name of namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='False',
        description='Usar namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Usar reloj de simulación (Gazebo)')
        
    declare_use_lsm_cmd = DeclareLaunchArgument(
        'use_lsm', default_value='False',
        description='Activar Laser Scan Matcher')

    declare_use_controller_cmd = DeclareLaunchArgument(
        'use_controller', default_value='False',
        description='Activar el controlador para trayectorias')

    declare_use_amcl_cmd = DeclareLaunchArgument(
        'use_amcl', default_value='False',
        description='Utilizar el amcl para el control')
    
    declare_use_aruco_cmd = DeclareLaunchArgument(
        'use_aruco', default_value='False',
        description='Utilizar el amcl para el control')

    declare_use_fusion_amcl_odom_cmd = DeclareLaunchArgument(
        'use_fusion_amcl_odom', default_value='True',
        description='Utilizar fusion entre amcl y odometria para el control del leader robot')
    
    declare_use_fusion_aruco_odom_cmd = DeclareLaunchArgument(
        'use_fusion_aruco_odom', default_value='True',
        description='Utilizar fusion entre aruco y odometria para el control del follower robot')

    declare_use_broadcast_ref_cmd = DeclareLaunchArgument(
        'use_broadcast_ref', default_value='False',
        description='Activar transmisión de la trayectoria de referencia')
        
    declare_use_phi_cmd = DeclareLaunchArgument(
        'use_phi', default_value='True',
        description='Activar trayectoria con el parametro phi para velocidad constante')
    
    declare_use_close_loop_cmd = DeclareLaunchArgument(
        'use_close_loop', default_value='True',
        description='True para activar el control en lazo cerrado')

    declare_controller_type_cmd = DeclareLaunchArgument(
        'controller_type',
        default_value='Kp',
        description='File name to choose between diferents types of controllers')
    
    declare_velocity_cmd = DeclareLaunchArgument(
        'velocity',
        default_value='0.1',
        description='Robot Velocity')
    
    declare_Ax_cmd = DeclareLaunchArgument(
        'Ax',
        default_value='2.5',
        description='Trajectory Amplitude on x axis')
    
    declare_Ay_cmd = DeclareLaunchArgument(
        'Ay',
        default_value='0.81',
        description='Trajectory Amplitude on y axis')
    
    declare_distance_cmd = DeclareLaunchArgument(
        'distance',
        default_value='0.8',
        description='Distance between the robots')
    
    declare_phi0_cmd = DeclareLaunchArgument(
        'phi0',
        default_value='-0.275',
        description='Phi from the follower robot')
    
    declare_period_cmd = DeclareLaunchArgument(
        'period',
        default_value='0.125',
        description='Period of the control signal')

    #Acciones a llevar a cabo por el launch
    actions = [
        #Coloca las acciones bajo el namespace
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        # Incluir los launch del laser y robot_state_publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([localization_launch_file]),
            condition=IfCondition(use_amcl)), #laser 

 
        # Nodo de control del robot para realizar trayectorias
        Node(
            condition=IfCondition(use_controller),  # Condición de uso del controlador
            package='central_pkg',
            executable='central_control_node',
            name='central_control_node',
            parameters=[{'use_sim_time': use_sim_time},
                        {'use_lsm': use_lsm},
                        {'use_amcl': use_amcl},
                        {'use_aruco': use_aruco},
                        {'use_fusion_amcl_odom': use_fusion_amcl_odom},
                        {'use_fusion_aruco_odom': use_fusion_aruco_odom},
                        {'use_broadcast_ref': use_broadcast_ref},
                        {'use_phi': use_phi},
                        {'use_close_loop': use_close_loop},
                        {'controller_type': controller_type},
                        {'velocity': velocity},
                        {'Ax': Ax},
                        {'Ay': Ay},
                        {'distance': distance},
                        {'phi0': phi0},
                        {'period': period}],
                        
            output='screen',
            respawn_delay=1.0,
            emulate_tty=True, 
        ),
    ]

    bringup_robot_cmd_group = GroupAction(actions)

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_lsm_cmd)
    ld.add_action(declare_use_controller_cmd)
    ld.add_action(declare_use_amcl_cmd)
    ld.add_action(declare_use_aruco_cmd)
    ld.add_action(declare_use_fusion_amcl_odom_cmd)
    ld.add_action(declare_use_fusion_aruco_odom_cmd)
    ld.add_action(declare_use_broadcast_ref_cmd)
    ld.add_action(declare_use_phi_cmd)
    ld.add_action(declare_use_close_loop_cmd)
    ld.add_action(declare_controller_type_cmd)
    ld.add_action(declare_velocity_cmd)
    ld.add_action(declare_Ax_cmd)
    ld.add_action(declare_Ay_cmd)
    ld.add_action(declare_distance_cmd)
    ld.add_action(declare_phi0_cmd)
    ld.add_action(declare_period_cmd)

    ld.add_action(bringup_robot_cmd_group)

    return ld
