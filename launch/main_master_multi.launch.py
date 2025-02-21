import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node,PushRosNamespace

from launch.conditions import IfCondition

from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    #Declaracion de variables tomadas de las variables de entorno:  
    ROBOT_NAMESPACE = os.environ['ROBOT_NAMESPACE']
    
    #Declaracion de la nueva variable para el fichero yaml:
    waffle_pi_filename ='waffle_pi_'+ROBOT_NAMESPACE+'.yaml'

    #Nombre del nuevo paquete del workspace multiagent:
    pkg_turtlebot3_multi = get_package_share_directory('tb30_multi_pkg')

    #Asignacion de los Parametros de configuracion que vamos a declarar
    use_tf_local = LaunchConfiguration('use_tf_local')
    param_file = LaunchConfiguration('param_file')
    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    #Instancia un objeto con la ruta al fichero yaml con los parametros de inicializacion.
    #Este tipo de instancia nos permite introducir mediante linea de comandos la ruta del fichero yaml
    param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=PathJoinSubstitution(
            [pkg_turtlebot3_multi, 'param',waffle_pi_filename]),
        description='Turtlebot3 Robot param file'
    )

    #Instancia para modificar ficheros yaml (en este caso para añadir el namespace)
    namespaced_param_file = RewrittenYaml(
        source_file=param_file, #fichero que queremos modificar
        root_key=ROBOT_NAMESPACE, #lo que queremos añadir a los parametros del fichero (en este caso el namespace)
        param_rewrites={},  #MCambios que queremos realizar en los parametros en sí mismos
        convert_types=True) #Lo usamos para convertir los valores de tipo string del fichero en los float, int.. que necesita el codigo


    # Instancias con la ruta a otros launch
    ld08_launch_file = PathJoinSubstitution(
        [pkg_turtlebot3_multi, 'launch', 'ld08.launch.py']) #launch del laser

    turtlebot3_state_publisher_launch_file = PathJoinSubstitution(
        [pkg_turtlebot3_multi, 'launch', 'turtlebot3_state_publisher.launch.py'] #launch del robot_state_publisher
    )

    #Acciones que pasaremos al metodo add_action
    actions = [
            #Añadir el namespace a los nodos que se lanzaran con esta accion
            PushRosNamespace(ROBOT_NAMESPACE),

            #Lanzamiento de los launch del laser y del robot_state_publisher
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ld08_launch_file])), #laser

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([turtlebot3_state_publisher_launch_file])), #robot_state_publisher
            
            #Nodo de inicializacion del robot
            #Uncomment remapping para tf_local / Comment para tf_global
            Node(
                package='turtlebot3_node',
                executable='turtlebot3_ros',
                name='turtlebot3_ros',
                parameters=[namespaced_param_file],
                arguments=['-i', usb_port],
                output='screen',
                #remappings=[
                #('/tf', 'tf'),
                #('/tf_static', 'tf_static')]
                ),
        ]
    
    #Creamos una instancia de las acciones que queremos realizar
    turtlebot3_standard = GroupAction(actions)

    #Instanciamos un objeto de la clase LaunchDescription que sera lo que devuelva 
    #la funcion generate_launch_description
    ld = LaunchDescription()

    #Llamamos al metodo add_action con el objeto creado que queremos poder pasarle como argumento en la linea de comandos
    #es imortante llamar primero a este metodo con la instancia de los parametros para que cargue el valor por default o por terminal
    ld.add_action(param_file_cmd)

    #Llamamos al metodo add_action para configurar como acciones a realizar las definidas dentro de "actions"
    ld.add_action(turtlebot3_standard)

    return ld