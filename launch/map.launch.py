import os
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    package_path = get_package_share_directory('patrolling_sim_ros2')
    map_name = LaunchConfiguration('map').perform(context)

    stage_node = Node(package='stage_ros2',
                      namespace='/',
                      executable='stage_ros2',
                      parameters=[{"world": os.path.join(package_path, 'maps', map_name, map_name+'.world')}])

    return [
        stage_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map'),
        OpaqueFunction(function=launch_setup)
    ])


generate_launch_description()
