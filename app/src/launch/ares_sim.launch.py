"""
ARES Simulation Launch File
============================
Starts: Gazebo (with ares_alex_default_world.sdf) + MAVROS + selected controller + metrics logger

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
  ros2 launch app ares_sim.launch.py                      # NMPC (default)
  ros2 launch app ares_sim.launch.py controller:=nmpc
  ros2 launch app ares_sim.launch.py controller:=nrhdg

Optional arguments:
  controller:=<nmpc|nrhdg>   Controller to run (default: nmpc)
  fcu_url:=<url>             FCU connection URL (default: tcp://127.0.0.1:5762)
"""

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction


def generate_launch_description():
    pkg_share = get_package_share_directory('app')

    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='nmpc',
        description='Controller to run: nmpc or nrhdg',
    )

    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='tcp://127.0.0.1:5762',
        description='ArduPilot SITL MAVLink connection URL',
    )

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
            'plugin_allowlist': [
                'sys_status',
                'local_position',
                'setpoint_raw',
                'command',
                'param',
            ],
        }],
    )

    use_nmpc  = IfCondition(PythonExpression(
        ["'", LaunchConfiguration('controller'), "' == 'nmpc'"]
    ))
    use_nrhdg = IfCondition(PythonExpression(
        ["'", LaunchConfiguration('controller'), "' == 'nrhdg'"]
    ))

    # --- NMPC controller node (launched when controller:=nmpc) ---
    nmpc_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='app',
                executable='nmpc_node',
                name='nmpc_controller',
                output='screen',
                condition=use_nmpc,
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

    # --- NRHDG controller node (launched when controller:=nrhdg) ---
    nrhdg_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='app',
                executable='nrhdg_node',
                name='nrhdg_controller',
                output='screen',
                condition=use_nrhdg,
                parameters=[{
                    'horizon':         20,
                    'dt':              0.05,
                    'Q_pos':           10.0,
                    'Q_vel':           1.0,
                    'Q_angle':         5.0,
                    'Q_rate':          0.5,
                    'R_thrust':        0.01,
                    'R_torque':        0.1,
                    'thrust_max':      30.0,
                    'torque_max':      2.0,
                    'angle_max':       0.52,
                    'gamma':           2.0,
                    'w_max':           1.0,
                    'game_iters':      3,
                    'game_tol':        1e-3,
                    'solver_max_iter': 100,
                }],
            )
        ],
    )

    # --- Metrics logger: always launched, subscribes to active controller topic ---
    metrics_logger = TimerAction(
        period=5.5,
        actions=[
            Node(
                package='app',
                executable='metrics_logger_node',
                name='metrics_logger',
                output='screen',
                parameters=[{
                    'controller': LaunchConfiguration('controller'),
                }],
            )
        ],
    )

    return LaunchDescription([
        ardupilot_plugin_path,
        controller_arg,
        fcu_url_arg,
        mavros_node,
        nmpc_node,
        nrhdg_node,
        metrics_logger,
    ])
