"""Standalone calibrated velocity interface. Hardware is opt-in; local Arm required."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    share=Path(get_package_share_directory('whill_controller'))
    boolean=lambda name:ParameterValue(LaunchConfiguration(name),value_type=bool)
    controller=Node(package='whill_controller',executable='controller_node',name='whill_controller',
        parameters=[{'hardware_enabled':boolean('hardware_enabled'),'serial_port':LaunchConfiguration('serial_port'),
          'whill_mode':True,'keyboard_auto_drag':True,'fine_hid_enabled':boolean('fine_hid_enabled'),
          'eye_control_enabled':False,'navigation_control_enabled':True,'navigation_continuous_mapping':True,
          'native_nav2_mode':True,'navigation_allow_reverse':boolean('allow_reverse'),'navigation_hold_to_run':False,
          'navigation_latched_arm':boolean('latched_arm'),
          'unified_ui':boolean('unified_ui'),
          'rviz_goal_authority_enabled':False,'calibration_file':LaunchConfiguration('calibration_file')}],output='screen')
    adapter=Node(package='whill_controller',executable='cmd_vel_adapter',name='whill_cmd_vel_adapter',
        parameters=[{'cmd_vel_topic':LaunchConfiguration('cmd_vel_topic'),'stamped_input':boolean('stamped_input')}],output='screen')
    def stop(event,context):return [] if context.is_shutdown else [EmitEvent(event=Shutdown(reason='Velocity interface process exited'))]
    return LaunchDescription([
      DeclareLaunchArgument('hardware_enabled',default_value='false'),
      DeclareLaunchArgument('serial_port',default_value='/dev/ble2wchair'),
      DeclareLaunchArgument('cmd_vel_topic',default_value='/cmd_vel'),
      DeclareLaunchArgument('stamped_input',default_value='false'),
      DeclareLaunchArgument('fine_hid_enabled',default_value='true'),
      DeclareLaunchArgument('allow_reverse',default_value='true'),
      DeclareLaunchArgument('latched_arm',default_value='false'),
      DeclareLaunchArgument('unified_ui',default_value='false'),
      DeclareLaunchArgument('calibration_file',default_value=str(share/'config/ipad_calibration_20261009.json')),
      RegisterEventHandler(OnProcessExit(target_action=controller,on_exit=stop)),
      RegisterEventHandler(OnProcessExit(target_action=adapter,on_exit=stop)),controller,adapter])
