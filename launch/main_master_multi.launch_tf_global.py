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

    namespace = EnvironmentVariable('ROBOT_NAMESPACE', default_value='')
    model = EnvironmentVariable('TURTLEBOT3_MODEL', default_value='')

    ROBOT_NAMESPACE = os.environ['ROBOT_NAMESPACE']


    #pkg_turtlebot3_bringup = get_package_share_directory('turtlebot3_bringup')
    pkg_turtlebot3_multi = get_package_share_directory('tb30_multi_pkg')
    pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')
    #pkg_nav2 = get_package_share_directory('nav2_bringup')
    
    #if (namespace==''):
    #    waffle_pi_filename='waffle_pi.yaml'
    #else:
    #    waffle_pi_filename='waffle_pi_'+ROBOT_NAMESPACE+'.yaml'

    waffle_pi_filename='waffle_pi_'+ROBOT_NAMESPACE+'.yaml'

    param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=PathJoinSubstitution(
            [pkg_turtlebot3_multi, 'param',waffle_pi_filename]),
        description='Turtlebot3 Robot param file'
    )

    use_odom_topic_cmd = DeclareLaunchArgument(
        'use_odom_topic',
        default_value='False',
        description='This parameter determines to use the odometry topic instead of the transform listener through tf topic'
    )

    fixed_frame_cmd = DeclareLaunchArgument(
        'fixed_frame',
        default_value='map',
        description='This parameter sets the fixed frame id when using the transform listener from base_link to this frame id'
    )

    base_link_frame_cmd = DeclareLaunchArgument(
        'base_link_frame',
        default_value='base_link',
        description='This parameter sets the base_link frame id when using the transform listener from this frame id to fixed frame id'
    )

    param_file = LaunchConfiguration('param_file')
    use_odom_topic = LaunchConfiguration('use_odom_topic')
    fixed_frame = LaunchConfiguration('fixed_frame')
    base_link_frame = LaunchConfiguration('base_link_frame')
    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    namespaced_param_file = RewrittenYaml(
        source_file=param_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=True)


    
        # Launch files
    #turtlebot3_robot_launch_file = PathJoinSubstitution([pkg_turtlebot3_bringup, 'launch', 'robot.launch.py'])
    ld08_launch_file = PathJoinSubstitution(
        [pkg_turtlebot3_multi, 'launch', 'ld08.launch.py'])

    #amcl_launch_file = PathJoinSubstitution(
    #    [pkg_nav2, 'launch', 'localization_launch.py'])

    turtlebot3_state_publisher_launch_file = PathJoinSubstitution(
        [pkg_turtlebot3_multi, 'launch', 'turtlebot3_state_publisher.launch.py']
    )

    #print(PathJoinSubstitution([pkg_turtlebot3_multi, 'param', 'waffle_pi.yaml']))


    actions = [
            PushRosNamespace(namespace),
            #IncludeLaunchDescription(
            #    PythonLaunchDescriptionSource([turtlebot3_robot_launch_file]),
            #    launch_arguments=[('model', 'waffle_pi'),
            #                      ('param_file', namespaced_param_file)]),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ld08_launch_file])),

            #IncludeLaunchDescription(
            #    PythonLaunchDescriptionSource([amcl_launch_file])),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([turtlebot3_state_publisher_launch_file])),
            Node(
                package='turtlebot3_node',
                executable='turtlebot3_ros',
                name='turtlebot3_ros',
                parameters=[namespaced_param_file],
                arguments=['-i', usb_port],
                output='screen',
                
                ),
            #Node(
            #    condition=IfCondition(use_odom_topic),
            #    package='tb30_multi_pkg',
            #    executable='control_trajectory',
            #    name='control_trajectory',
            #    parameters=[{'use_odom_topic': True}]
            #    ),
            #Node(
            #    condition=IfCondition(PythonExpression(['not ', use_odom_topic])),
            #    package='tb30_multi_pkg',
            #    executable='control_trajectory',
            #    name='control_trajectory',
            #    parameters=[{'use_odom_topic': False},{'fixed_frame':fixed_frame},{'base_link_frame':base_link_frame}]
            #    ),

            Node(
                package='tb30_multi_pkg',
                executable='broadcaster_reference_0',
                name='broadcaster_reference_0',
                ),
        ]
    
    turtlebot3_standard = GroupAction(actions)

    ld = LaunchDescription()
    ld.add_action(param_file_cmd)
    ld.add_action(use_odom_topic_cmd)
    ld.add_action(fixed_frame_cmd)
    ld.add_action(base_link_frame_cmd)
    ld.add_action(turtlebot3_standard)
    return ld