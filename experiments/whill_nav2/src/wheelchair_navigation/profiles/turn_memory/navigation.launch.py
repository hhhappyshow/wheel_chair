from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    here = Path(__file__).resolve().parent
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(here/'navigation.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_map_adapter', default_value='true'),
        Node(package='laser_filters', executable='scan_to_scan_filter_chain',
             name='nav2_self_mask',
             parameters=[str(here/'self_mask.yaml'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}],
             remappings=[('scan','/wheelchair/scan'),
                         ('scan_filtered','/wheelchair/nav2/scan_masked')],
             output='screen'),
        Node(package='laser_filters', executable='scan_to_scan_filter_chain',
             name='scan_to_scan_filter_chain',
             parameters=[str(here/'memory_scan_filter.yaml'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}],
             remappings=[('scan','/wheelchair/nav2/scan_masked'),('scan_filtered','/wheelchair/memory_scan')],
             output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            here.parents[1]/'launch/navigation_dense_candidate.launch.py')),
            launch_arguments={'params_file':LaunchConfiguration('params_file'),
                              'use_sim_time':LaunchConfiguration('use_sim_time'),
                              'start_map_adapter':LaunchConfiguration('start_map_adapter'),
                              'scan_topic':'/wheelchair/nav2/scan_masked',
                              'connect_actuator':'false','start_domain_preview':'false','rviz':'false'}.items())
    ])
