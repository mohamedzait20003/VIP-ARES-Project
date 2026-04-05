"""
ARES Simulation Launch File
============================
Starts: Gazebo (with ares_alex_default_world.sdf) + MAVROS + NMPC node

Prerequisites:
  1. ardupilot_gazebo plugin installed and on GZ_SIM_SYSTEM_PLUGIN_PATH
       https://github.com/ArduPilot/ardupilot_gazebo
  2. ArduPilot SITL started separately (before or after this launch):
       cd ~/ardupilot
       sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
  3. ROS2 Jazzy workspace sourced:
       source ~/ros2_ws/install/setup.bash
  4. MAVROS installed:
       sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras

Usage:
  ros2 launch app ares_sim.launch.py

Optional arguments:
  fcu_url:=<url>   FCU connection URL (default: udp://127.0.0.1:14550@14555)
  world:=<path>    Absolute path to SDF world file (default: auto-detected)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('app')

    # Default world path — installed via setup.py data_files
    default_world = os.path.join(pkg_share, 'worlds', 'ares_alex_default_world.sdf')

    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='tcp://127.0.0.1:5762',
        description='ArduPilot SITL MAVLink connection URL',
    )

    # Ensure Gazebo can find the ArduPilotPlugin regardless of shell env
    ardupilot_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=os.path.join(os.path.expanduser('~'), 'ardupilot_gazebo', 'build'),
    )

    # --- MAVROS: bridges ArduPilot MAVLink <-> ROS2 ---
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': '',
            'target_system_id': 1,
            'target_component_id': 1,
            'fcu_protocol': 'v2.0',
            # Only load plugins we need — avoids type-conflict crashes on Jazzy
            'plugin_allowlist': [
                'sys_status',
                'local_position',
                'setpoint_raw',
                'command',
                'param',
            ],
        }],
    )

    # --- NMPC controller node ---
    # Delayed slightly to allow MAVROS to connect before the control loop starts
    nmpc_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='app',
                executable='nmpc_node',
                name='nmpc_controller',
                output='screen',
                parameters=[{
                    'horizon':    20,
                    'dt':         0.05,
                    'Q_pos':      10.0,
                    'Q_vel':      1.0,
                    'Q_angle':    5.0,
                    'Q_rate':     0.5,
                    'R_thrust':   0.01,
                    'R_torque':   0.1,
                    'thrust_max': 30.0,
                    'torque_max': 2.0,
                    'angle_max':  0.52,
                }],
            )
        ],
    )

    return LaunchDescription([
        ardupilot_plugin_path,
        fcu_url_arg,
        mavros_node,
        nmpc_node,
    ])
