#!/usr/bin/env python3
#
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString, RewrittenYaml
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('patrolling_sim_ros2')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    pose = {'x': LaunchConfiguration('x_pose'),
            'y': LaunchConfiguration('y_pose'),
            'yaw': LaunchConfiguration('yaw_pose')}
    use_amcl = LaunchConfiguration('use_amcl')

    remappings = [('map', '/map')]

    # Create our own temporary YAML files that include substitutions
    # param_substitutions = {
    #     'use_sim_time': use_sim_time,
    #     #'yaml_filename': map_yaml_file,
    #     'amcl.ros__parameters.initial_pose.x': pose['x'],
    #     'amcl.ros__parameters.initial_pose.y': pose['y'],
    #     'amcl.ros__parameters.initial_pose.y': pose['yaw']}
    param_substitutions = {}

    params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': namespace},
    )

    configured_params = RewrittenYaml(
        source_file=params_file,
        #root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'use_amcl', default_value='true',
            description='Use AMCL algorithm if true'),

        DeclareLaunchArgument(
            'use_amcl', default_value='true',
            description='Use AMCL algorithm if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                bringup_dir, 'params', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                configured_params,
                {
                    'initial_pose': {
                        'x':    pose['x'],
                        'y':    pose['y'],
                        'z':    0.0,
                        'yaw':  pose['yaw']
                     },
                     'use_sim_time': True
                }
            ],
            remappings=remappings,
            condition=IfCondition(use_amcl)
        ),

        Node(
            package = 'fake_localization_ros2',
            executable='fake_localization',
            name='fake_localization',
            parameters=[
                configured_params,
                {
                    'delta_x':          pose['x'],
                    'delta_y':          pose['y'],
                    'delta_yaw':        pose['yaw']
                }
            ],
            condition=UnlessCondition(use_amcl)
        ),

        # ** The following node isn't needed because autostart_node parameter of amcl node is TRUE in yaml file
        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_localization',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time},
        #                 {'autostart': autostart},
        #                 {'node_names': ['amcl']
        #                 }],
        #     condition=IfCondition(use_amcl))
    ])
