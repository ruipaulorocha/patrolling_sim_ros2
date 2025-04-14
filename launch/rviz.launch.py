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
        description='Absolute path of world file'
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value=TextSubstitution(text =  'info'),
        description='Log level'
    )

    ld.add_action(stage_world_arg)
    ld.add_action(log_level_arg)

    # opaque function
    def launch_nodes(context):
        log_level = context.launch_configurations['log_level']
        this_pkg_directory = get_package_share_directory('patrolling_sim_ros2')
        rviz_config = os.path.join(this_pkg_directory, 'config',
             context.launch_configurations['world'] + '.rviz')

        return [
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                parameters=[{'use_sim_time': True}],
                arguments=['-d', rviz_config, '--ros-args', '--log-level', log_level]
            )
        ]

    launch_nodes_fn = OpaqueFunction(function=launch_nodes)

    # specify the actions
    ld.add_action(launch_nodes_fn)

    return ld
