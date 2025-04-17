#!/usr/bin/env python3
#
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
import configparser

initPoses = {}
package_path = get_package_share_directory('patrolling_sim_ros2')
delays = [4,8,12,16,20,24,28,32,36,40,44,48]

def loadInitPoses():
    try:
        ConfigIP = configparser.ConfigParser()
        ConfigIP.read(package_path+"/config/initial_poses.txt")
        for option in ConfigIP.options("InitialPoses"):
            initPoses[option] = ConfigIP.get("InitialPoses", option)
    except:
        print("Could not load initial poses file")

def launch_setup(context, *args, **kwargs):
    # Get the launch directory
    launch_dir = os.path.join(package_path, 'launch')

    # Names and poses of the robots
    loadInitPoses()

    map_name_str = LaunchConfiguration('map').perform(context)
    n_robots_str = LaunchConfiguration('n_robots').perform(context)
    n_robots = int(n_robots_str)

    scenario = map_name_str+'_'+n_robots_str
    iposes = initPoses[scenario.lower()]
    iposes = iposes.replace('[','')
    iposes = iposes.replace(']','')
    iposes = iposes.split(',')

    robots = []
    for i in range(n_robots):
        robots.append({'name': ('robot_'+str(i)), 'x_pose': iposes[i*2], 'y_pose': iposes[i*2+1],
        'z_pose': 0.01, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0})
    print(robots)

    # Simulation settings
    simulator = LaunchConfiguration('simulator')

    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')
    log_settings = LaunchConfiguration('log_settings', default='true')

    # Declare the launch arguments
    world_str = package_path +'/worlds/' + map_name_str + '.world'

    # declare_map_yaml_cmd = DeclareLaunchArgument(
    #     'map_yaml',
    #     default_value=os.path.join(package_path, 'maps','1r5', '1r5.yaml'),
    #     description='Full path to map file to load')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(package_path, 'rviz', 'nav2_namespaced_view.rviz'),
        description='Full path to the RVIZ config file to use.')

    # Define commands for launching the navigation instances
    nav_instances_cmds = []

    map_yaml = package_path+'/maps/'+map_name_str+'/'+map_name_str+'.yaml'

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': True,
            'autostart_node': True
        }]
    )
    nav_instances_cmds.append(map_server_node)

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        arguments=['-d' + context.launch_configurations['rviz_config']],
        condition=IfCondition(use_rviz)
    )
    nav_instances_cmds.append(rviz2_node)


    i = 0;
    params_file = os.path.join(package_path, 'config', 'nav2-generic_namespace.yaml')

    for robot in robots:
        #params_file = LaunchConfiguration(f"{robot['name']}_params_file")

        group = GroupAction([
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #             os.path.join(launch_dir, 'rviz_launch.py')),
            #     condition=IfCondition(use_rviz),
            #     launch_arguments={
            #                       'namespace': TextSubstitution(text=robot['name']),
            #                       'use_namespace': 'True',
            #                       'rviz_config': rviz_config_file}.items()),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(package_path,
                                                           'launch',
                                                           'stop_simulation_launch.py')),
                launch_arguments={'namespace': robot['name'],
                                  'use_namespace': 'True',
                                  'map': package_path+'/maps/'+map_name_str+'/'+map_name_str+'.yaml',
                                  'use_sim_time': 'True',
                                  'params_file': params_file,
                                  'autostart': autostart,
                                  'use_amcl': LaunchConfiguration('use_amcl'),
                                  'use_rviz': 'False',
                                  'use_sim_time': 'True',
                                  'headless': 'False',
                                  'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                                  'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                                  'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                                  'roll': TextSubstitution(text=str(robot['roll'])),
                                  'pitch': TextSubstitution(text=str(robot['pitch'])),
                                  #'yaw': TextSubstitution(text=str(robot['yaw'])),
                                  # initial yaw is fixed for every robot in current version and is =PI/2
                                  'yaw': TextSubstitution(text=str(1.570796327)),
                                  'robot_name':TextSubstitution(text=robot['name']), }.items()),
            #Node(
            #    packag='tf_demux',
            #    executable='tf_demux',
            #    namespace=robot['name'],
            #    parameters=[{"robot_prefix": robot['name'],
            #                "use_sim_time": True}]),

            LogInfo(
                condition=IfCondition(log_settings),
                msg=['Launching ', robot['name']]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' map yaml: ', map_name_str]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' params yaml: ', params_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' rviz config file: ', rviz_config_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' autostart: ', autostart])
        ])
        timed_action = TimerAction(period=float(delays[i]),actions=[group])
        i = i + 1;

        nav_instances_cmds.append(timed_action)

    nav_instances_cmds.append(declare_rviz_config_file_cmd)

    return nav_instances_cmds


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='True',description='Automatically startup the stacks'),
        DeclareLaunchArgument('use_rviz',default_value='True',description='Whether to start RVIZ'),
        DeclareLaunchArgument('rviz_config',default_value=os.path.join(package_path, 'rviz', 'nav2_namespaced_view.rviz'),description='Full path to the RVIZ config file to use.'),
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('n_robots'),
        DeclareLaunchArgument('use_amcl', default_value='True', description='Use AMCL algorithm if true'),
        OpaqueFunction(function=launch_setup)
    ])

generate_launch_description()
