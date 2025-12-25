#!/usr/bin/env python3
"""
FIXED Gate Launch File
All issues resolved - proper spawn depth, timings, and node initialization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
import launch_ros.descriptions


def generate_launch_description():
    auv_slam_share = get_package_share_directory('auv_slam')
    
    # Paths
    urdf_file = os.path.join(auv_slam_share, 'urdf', 'orca4_description.urdf')
    bridge_config = os.path.join(auv_slam_share, 'config', 'ign_bridge.yaml')
    thruster_params = os.path.join(auv_slam_share, 'config', 'thruster_params.yaml')
    world_file = os.path.join(auv_slam_share, 'worlds', 'video_world.sdf')
    
    # Gazebo environment setup
    gz_models_path = os.path.join(auv_slam_share, "models")
    gz_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", default="")
    gz_env = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH':
           ':'.join([os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                     os.environ.get('LD_LIBRARY_PATH', default='')]),
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':  
           ':'.join([os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                     os.environ.get('LD_LIBRARY_PATH', default='')]),
        'GZ_SIM_RESOURCE_PATH':
           ':'.join([gz_resource_path, gz_models_path])
    }
    
    # ========================================================================
    # CORE SIMULATION NODES
    # ========================================================================
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': launch_ros.descriptions.ParameterValue(
                Command(['xacro ', urdf_file]), value_type=str
            ),
            'use_sim_time': True
        }]
    )
    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    gazebo_process = ExecuteProcess(
        cmd=['ruby', FindExecutable(name="ign"), 'gazebo', '-r', '-v', '3', world_file],
        output='screen',
        additional_env=gz_env,
        shell=False
    )
    
    # CRITICAL FIX: Spawn at -0.5m depth (matches navigator target)
    # Previous: -0.5 (correct)
    # Position: X=-4.5m (4m from gate at X=-0.5m)
    spawn_entity = TimerAction(
        period=1.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-name", "orca4_ign",
                    "-topic", "robot_description",
                    "-z", "-0.5",      # MATCHES navigator target depth
                    "-x", "-4.5",      # 4m from gate
                    "-y", "0.0",
                    "-Y", "0.0",
                    "--ros-args", "--log-level", "warn"
                ],
                parameters=[{"use_sim_time": True}],
            )
        ]
    )
    
    # ROS-Gazebo Bridge
    bridge = TimerAction(
        period=1.5,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    '--ros-args',
                    '-p', f'config_file:={bridge_config}'
                ],
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # Thruster Mapper
    thruster_mapper = TimerAction(
        period=1.5,
        actions=[
            Node(
                package='auv_slam',
                executable='simple_thruster_mapper.py',
                name='thruster_mapper',
                output='screen',
                parameters=[thruster_params, {'use_sim_time': True}]
            )
        ]
    )
    
    # FIXED White Gate Detector (HSV-based)
    gate_detector = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='auv_slam',
                executable='white_gate_detector.py',
                name='simple_gate_detector',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # FIXED White Gate Navigator (proper depth control)
    gate_navigator = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='auv_slam',
                executable='white_gate_navigator.py',
                name='simple_gate_navigator',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # Debug Image Viewer
    debug_viewer = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='rqt_image_view',
                executable='rqt_image_view',
                name='debug_viewer',
                arguments=['/simple_gate/debug_image'],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        gazebo_process,
        spawn_entity,      # 1s delay - spawn at correct depth
        bridge,            # 1.5s delay
        thruster_mapper,   # 1.5s delay
        gate_detector,     # 2s delay - FIXED detector
        gate_navigator,    # 2s delay - FIXED navigator
        debug_viewer,      # 2s delay
    ])


if __name__ == '__main__':
    generate_launch_description()