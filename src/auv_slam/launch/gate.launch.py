#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
import launch_ros.descriptions


def generate_launch_description():
    auv_slam_share = get_package_share_directory('auv_slam')
    
    urdf_file = os.path.join(auv_slam_share, 'urdf', 'orca4_description.urdf')
    bridge_config = os.path.join(auv_slam_share, 'config', 'ign_bridge.yaml')
    thruster_params = os.path.join(auv_slam_share, 'config', 'thruster_params.yaml')
    world_file = os.path.join(auv_slam_share, 'worlds', 'video_world.sdf')
    
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
    
    spawn_entity = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-name", "orca4_ign",
                    "-topic", "robot_description",
                    "-z", "-0.5",
                    "-x", "-4.5",
                    "-y", "0.0",
                    "-Y", "0.0",
                    "--ros-args", "--log-level", "warn"
                ],
                parameters=[{"use_sim_time": True}],
            )
        ]
    )
    
    bridge = TimerAction(
        period=3.0,
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
    
    thruster_mapper = TimerAction(
        period=3.0,
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
    
    gate_detector = TimerAction(
        period=5.0,
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
    
    gate_navigator = TimerAction(
        period=5.0,
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
    
    debug_viewer = TimerAction(
        period=8.0,
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
        spawn_entity,
        bridge,
        thruster_mapper,
        gate_detector,
        gate_navigator,
        debug_viewer,
    ])


if __name__ == '__main__':
    generate_launch_description()