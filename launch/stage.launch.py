#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import (DeclareLaunchArgument, OpaqueFunction,
        IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    # create the launch description and populate
    ld = LaunchDescription()

    # declare launch arguments
    stage_world_arg = DeclareLaunchArgument(
        'world',
        default_value=TextSubstitution(text =  'default'),
        description='Absolute path of world file')
    teamsize_arg = DeclareLaunchArgument(
        'teamsize',
        default_value=TextSubstitution(text =  '2'),
        description='Teamsize')
    remap_cmd_vel_flag_arg = DeclareLaunchArgument(
        'remap_cmd_vel',
        default_value=TextSubstitution(text =  'True'),
        description='Whether cmd_vel shall be remapped to cmd_vel_not_stamped')
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value=TextSubstitution(text =  'info'),
        description='Log level'
    )

    ld.add_action(stage_world_arg)
    ld.add_action(teamsize_arg)
    ld.add_action(remap_cmd_vel_flag_arg)
    ld.add_action(log_level_arg)

    # opaque function
    def launch_nodes(context):
        remap_flag = context.launch_configurations['remap_cmd_vel'].lower() in ['true', '1', 'yes']
        topic_name = 'cmd_vel_not_stamped' if remap_flag else 'cmd_vel'
        n = int(context.launch_configurations['teamsize'])
        log_level = context.launch_configurations['log_level']
        p3dx_description_directory = get_package_share_directory('p3dx_description_ros')
        remappings = []

        for i in range(n):
            remappings.append(
                (f'/robot_{i}/cmd_vel', f'/robot_{i}/{topic_name}' )
            )

        nodes = []

        stage_node = Node(
            package='stage_ros2',
            executable='stageros',
            name="stage",
            parameters=[{
                #'enable_gui': False,
                'base_watchdog_timeout': 1.0,
                'world_file': context.launch_configurations['world'],
                'odom_topic': 'odom',
                'use_sim_time': True
            }],
            remappings=remappings,
            arguments=['--ros-args', '--log-level', log_level]
        )
        nodes.append(stage_node)

        for i in range(n):
            namespace = 'robot_' + str(i)

            robot_description = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(p3dx_description_directory, 'launch',
                        'p3dx_description_ros2.launch.py')
                    ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'tf_prefix': namespace,
                    }.items()
            )
            nodes.append(robot_description)

            # convert stamped twist messages from topic cmd_vel to not stamped messages in topic cmd_vel_not_stamped
            twiststamped_to_twist_node = Node(
                package='patrolling_sim_ros2',
                executable='twist_stamped_to_twist',
                name='twist_stamped_to_twist',
                parameters=[{'use_sim_time': True}],
                namespace = namespace,
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                condition=IfCondition(str(remap_flag)),
            )
            nodes.append(twiststamped_to_twist_node)

        return nodes

    launch_nodes_fn = OpaqueFunction(function=launch_nodes)

    # specify the actions
    ld.add_action(launch_nodes_fn)

    return ld
