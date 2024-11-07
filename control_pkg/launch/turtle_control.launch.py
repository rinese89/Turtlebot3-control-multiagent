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
    ROBOT_NAMESPACE = os.environ['ROBOT_NAMESPACE']
    
    #Declaracion de la nueva variable para el fichero yaml:
    waffle_pi_filename ='waffle_pi.yaml'
    waffle_pi_filename_ns ='waffle_pi_'+ROBOT_NAMESPACE+'.yaml'

    #Asignacion de los parametros de lazamiento a unas instancias que utilizaremos despues en el código
    namespace =LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    usb_port = LaunchConfiguration('usb_port')
    use_lsm = LaunchConfiguration('use_lsm')
    use_aruco = LaunchConfiguration('use_aruco')
    use_aruco_no_servo = LaunchConfiguration('use_aruco_no_servo')
    use_controller = LaunchConfiguration('use_controller')
    use_amcl = LaunchConfiguration('use_amcl')
    use_fusion_amcl_odom = LaunchConfiguration('use_fusion_amcl_odom')
    use_broadcast_ref = LaunchConfiguration('use_broadcast_ref')
    use_phi = LaunchConfiguration('use_phi')
    use_sim_time = LaunchConfiguration('use_sim_time')
    matrix_file = LaunchConfiguration('matrix_file')

    #Rutas a los directorios de los paquetes
    this_pkg_dir = get_package_share_directory('control_pkg')

    #param_file_path = os.path.join(this_pkg_dir, 'param', waffle_pi_filename)
    param_file_path = os.path.join(
        this_pkg_dir, 'param', waffle_pi_filename)

    param_file_path_ns = os.path.join(
        this_pkg_dir, 'param', waffle_pi_filename_ns)
        

    namespaced_param_file = RewrittenYaml(
        source_file=param_file_path_ns, #fichero que queremos modificar
        root_key=ROBOT_NAMESPACE, #lo que queremos añadir a los parametros del fichero (en este caso el namespace)
        param_rewrites={},  #MCambios que queremos realizar en los parametros en sí mismos, ninguno en este caso
        convert_types=True)


    print(param_file_path)

    # Instancias con la ruta a otros launch
    ld08_launch_file = PathJoinSubstitution(
        [this_pkg_dir, 'launch', 'ld08.launch.py']) #launch del laser

    turtlebot3_state_publisher_launch_file = PathJoinSubstitution(
        [this_pkg_dir, 'launch', 'turtlebot3_state_publisher.launch.py'] #launch del robot_state_publisher
    )
       
    # Declarar argumentos de launch
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='tb3_1',
        description='Name of namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='False',
        description='Usar namespace')

    declare_usb_port_cmd = DeclareLaunchArgument(
        'usb_port',
        default_value='/dev/ttyACM0',
        description='Puerto que utilizaremos para la OpenCR')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Usar reloj de simulación (Gazebo)')
        
    declare_use_lsm_cmd = DeclareLaunchArgument(
        'use_lsm', default_value='False',
        description='Activar Laser Scan Matcher')
    
    declare_use_aruco_cmd = DeclareLaunchArgument(
        'use_aruco', default_value='False',
        description='Activar Aruco: camara y control del servo')

    declare_use_aruco_no_servo_cmd = DeclareLaunchArgument(
        'use_aruco_no_servo', default_value='False',
        description='Activar Aruco: camara sin control del servo')

    declare_use_controller_cmd = DeclareLaunchArgument(
        'use_controller', default_value='False',
        description='Activar el controlador para trayectorias')

    declare_use_amcl_cmd = DeclareLaunchArgument(
        'use_amcl', default_value='False',
        description='Utilizar el amcl para el control')

    declare_use_fusion_amcl_odom_cmd = DeclareLaunchArgument(
        'use_fusion_amcl_odom', default_value='True',
        description='Utilizar fusion entre amcl y odometria para el control')

    declare_use_broadcast_ref_cmd = DeclareLaunchArgument(
        'use_broadcast_ref', default_value='False',
        description='Activar transmisión de la trayectoria de referencia')
        
    declare_use_phi_cmd = DeclareLaunchArgument(
        'use_phi', default_value='True',
        description='Activar trayectoria con el parametro phi para velocidad constante')

    declare_matrix_file_cmd = DeclareLaunchArgument(
        'matrix_file',
        default_value='Kp.json',
        description='Matrix file name to the controller node')

    #Acciones a llevar a cabo por el launch
    actions = [
        #Coloca las acciones bajo el namespace
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        # Incluir los launch del laser y robot_state_publisher
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ld08_launch_file])), #laser

        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([turtlebot3_state_publisher_launch_file])), #robot_state_publisher

        Node(
            condition=IfCondition(use_namespace),
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            name='turtlebot3_ros',
            parameters = [namespaced_param_file],
            arguments=['-i', usb_port],
            output='screen',
            ),
        
        Node(
            condition=IfCondition(PythonExpression(['not ', LaunchConfiguration('use_namespace')])),
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters = [param_file_path],
            arguments=['-i', usb_port],
            output='screen',
            ),

        # Nodo para transmitir la trayectoria de referencia al tópico /tf y visualizarla en Rviz2
        Node(
            condition=IfCondition(use_broadcast_ref),  # Condición de uso del broadcaster
            package='control_pkg',
            executable='broadcaster_reference',
            name='broadcaster_reference',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen', 
        ),
       
        # Nodo para localizar el robot usando el paquete Laser Scan Matcher
        Node(
            condition=IfCondition(use_lsm),  # Condición de uso del laser_scan_matcher
            package='ros2_laser_scan_matcher',
            executable='laser_scan_matcher',
            name='laser_scan_matcher',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',  
        ),

        Node(
            condition=IfCondition(use_aruco),  # Condición de uso de la camara para la deteccion del aruco con servo
            package='control_pkg',
            executable='aruco_nav_node',
            name='aruco_nav_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen', 
        ),

        Node(
            condition=IfCondition(use_aruco_no_servo),  # Condición de uso de la camara para la deteccion del aruco sin servo
            package='control_pkg',
            executable='aruco_PID_node',
            name='aruco_PID_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen', 
        ),

        Node(
            condition=IfCondition(use_aruco),  # Condición de uso de la camara para la deteccion del aruco
            package='control_pkg',
            executable='camera_servo_node',
            name='camera_servo_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',  
        ),

 
        # Nodo de control del robot para realizar trayectorias
        Node(
            condition=IfCondition(use_controller),  # Condición de uso del controlador
            package='control_pkg',
            executable='controller_node',
            name='controller',
            parameters=[{'use_sim_time': use_sim_time},
                        {'use_lsm': use_lsm},
                        {'use_amcl': use_amcl},
                        {'use_fusion_amcl_odom': use_fusion_amcl_odom},
                        {'use_broadcast_ref': use_broadcast_ref},
                        {'use_phi': use_phi},
                        {'matrix_file': matrix_file}],
            #ros_arguments=['--ros-args', '-r', '__node:=controlador_new_name', '-r','/cmd_vel:=/command_vel'],     
            output='screen', 
        ),
    ]

    bringup_robot_cmd_group = GroupAction(actions)

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_usb_port_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_lsm_cmd)
    ld.add_action(declare_use_aruco_cmd)
    ld.add_action(declare_use_aruco_no_servo_cmd)
    ld.add_action(declare_use_controller_cmd)
    ld.add_action(declare_use_amcl_cmd)
    ld.add_action(declare_use_fusion_amcl_odom_cmd)
    ld.add_action(declare_use_broadcast_ref_cmd)
    ld.add_action(declare_use_phi_cmd)
    ld.add_action(declare_matrix_file_cmd)

    ld.add_action(bringup_robot_cmd_group)

    return ld
