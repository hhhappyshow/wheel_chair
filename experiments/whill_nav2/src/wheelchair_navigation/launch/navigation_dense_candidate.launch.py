"""Native navigation only; existing sensor drivers and hardware owner run separately."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = Path(get_package_share_directory('wheelchair_nav2_pipeline'))
    params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key='', param_rewrites={'use_sim_time': LaunchConfiguration('use_sim_time')},
        convert_types=True)
    servers = [
        ('nav2_planner', 'planner_server'),
        ('nav2_controller', 'controller_server'),
        ('nav2_smoother', 'smoother_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'),
        ('nav2_waypoint_follower', 'waypoint_follower'),
        ('nav2_velocity_smoother', 'velocity_smoother'),
        ('nav2_collision_monitor', 'collision_monitor'),
    ]
    nodes = []
    for package, executable in servers:
        remaps = []
        if executable in ('controller_server', 'behavior_server'):
            remaps = [('cmd_vel', '/wheelchair/nav2/cmd_vel_raw')]
        elif executable == 'velocity_smoother':
            remaps = [('cmd_vel', '/wheelchair/nav2/cmd_vel_raw'),
                      ('cmd_vel_smoothed', '/wheelchair/nav2/cmd_vel_smoothed')]
        node_params=[params]
        nodes.append(Node(package=package, executable=executable, name=executable,
                          parameters=node_params, remappings=remaps, output='screen'))
    actuator=Node(package='whill_controller',executable='controller_node',name='whill_controller',
                  parameters=[{'hardware_enabled':True,'serial_port':'/dev/ble2wchair','whill_mode':True,
                    'keyboard_auto_drag':True,'fine_hid_enabled':True,'eye_control_enabled':False,
                    'navigation_control_enabled':True,'navigation_continuous_mapping':True,
                    'native_nav2_mode':True,'navigation_allow_reverse':True,'navigation_hold_to_run':False,
                    'rviz_goal_authority_enabled':False}],condition=IfCondition(LaunchConfiguration('connect_actuator')),
                  output='screen')
    rviz=Node(package='rviz2',executable='rviz2',name='native_nav2_rviz',
              arguments=['-d',str(share/'config/pure_nav2.rviz')],condition=IfCondition(LaunchConfiguration('rviz')),output='screen')
    def shutdown_on_exit(event,context):
        return [] if context.is_shutdown else [EmitEvent(event=Shutdown(reason='Native Nav2 operator window closed'))]
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(share / 'config/navigation.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_map_adapter', default_value='false'),
        DeclareLaunchArgument('scan_topic', default_value='/wheelchair/scan'),
        DeclareLaunchArgument('connect_actuator', default_value='false'),
        DeclareLaunchArgument('start_domain_preview', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        RegisterEventHandler(OnProcessExit(target_action=actuator,on_exit=shutdown_on_exit)),
        RegisterEventHandler(OnProcessExit(target_action=rviz,on_exit=shutdown_on_exit)),
        actuator,rviz,
        *nodes,
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='native_navigation_lifecycle', parameters=[{
                 'autostart': True, 'node_names': [name for _, name in servers]}]),
        Node(package='wheelchair_hardware_bridge', executable='cloud_to_scan',
             parameters=[str(Path(get_package_share_directory('wheelchair_hardware_bridge')) /
                             'config/hardware_bridge.yaml')],
             condition=IfCondition(LaunchConfiguration('start_map_adapter'))),
        Node(package='wheelchair_hardware_bridge', executable='scan_map',
             parameters=[str(Path(get_package_share_directory('wheelchair_hardware_bridge')) /
                             'config/hardware_bridge.yaml')],
             remappings=[('/wheelchair/scan', LaunchConfiguration('scan_topic'))],
             condition=IfCondition(LaunchConfiguration('start_map_adapter'))),
        Node(package='wheelchair_nav2_pipeline', executable='rectangle_domains.py',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
             condition=IfCondition(LaunchConfiguration('start_domain_preview'))),
        Node(package='wheelchair_hardware_bridge', executable='twist_to_key',
             parameters=[{'source_topic': '/wheelchair/nav2/cmd_vel',
                          'target_topic': '/whill/controller/navigation',
                          'continuous_input': True}],
             condition=IfCondition(LaunchConfiguration('connect_actuator'))),
    ])
