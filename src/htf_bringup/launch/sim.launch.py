"""
HTF simulation bringup — run AFTER `make px4_sitl gz_f450` is already running.

Starts:
  1. MicroXRCEAgent     — bridges PX4 uORB topics to ROS 2 via uXRCE-DDS (UDP port 8888)
  2. robot_state_publisher — publishes TF tree from the F450 URDF
  3. ros_gz_bridge      — bridges Gazebo camera/clock topics to ROS 2

Usage:
  ros2 launch htf_bringup sim.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    pkg_f450 = get_package_share_directory('f450_description')
    pkg_bringup = get_package_share_directory('htf_bringup')

    xacro_file = os.path.join(pkg_f450, 'urdf', 'f450.urdf.xacro')
    bridge_config = os.path.join(pkg_bringup, 'config', 'gz_bridge.yaml')

    # ------------------------------------------------------------------
    # 0. Kill any leftover MicroXRCEAgent from a previous session.
    #    Port 8888 can only be bound once; an orphaned agent blocks the new one.
    #    '|| true' makes the command succeed even if no process was found.
    # ------------------------------------------------------------------
    kill_old_agent = ExecuteProcess(
        cmd=['bash', '-c', 'pkill -f MicroXRCEAgent || true'],
        output='screen',
        name='kill_old_agent',
    )

    # ------------------------------------------------------------------
    # 1. MicroXRCEAgent
    #    Connects to the uXRCE-DDS client already running inside PX4 SITL
    #    on UDP port 8888 and exposes /fmu/in|out/... topics to ROS 2.
    #    Delayed 3 s: 0.5 s for the pkill to clear the port + 2.5 s for SITL.
    # ------------------------------------------------------------------
    xrce_agent = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
                output='screen',
                name='micro_xrce_agent',
            )
        ]
    )

    # ------------------------------------------------------------------
    # 2. robot_state_publisher
    #    Reads the F450 URDF and publishes all fixed TF transforms so that
    #    camera_link, optical frames, and rotor frames are available in TF.
    # ------------------------------------------------------------------
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
            'use_sim_time': True,
        }]
    )

    # ------------------------------------------------------------------
    # 2b. joint_state_publisher
    #     Publishes zero joint states for the 4 continuous rotor joints.
    #     robot_state_publisher cannot resolve non-fixed joints without a
    #     /joint_states message — without this, RViz reports
    #     "No transform from [rotor_*] to [base_link]".
    #     Rotors appear stationary in RViz (acceptable: they spin at
    #     ~6000 RPM so animation adds no useful information).
    # ------------------------------------------------------------------
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ------------------------------------------------------------------
    # 3. ros_gz_bridge
    #    Bridges Gazebo simulation topics (camera feed, clock) to ROS 2.
    #    Config file lists every bridged topic so this file stays clean.
    # ------------------------------------------------------------------
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True,
        }]
    )

    return LaunchDescription([
        kill_old_agent,
        xrce_agent,
        rsp,
        jsp,
        gz_bridge,
    ])
