#!/usr/bin/env python3
"""
FIXED Mission Launch File - Same Terminal Order Input
Order prompt appears in the SAME terminal where you launch
"""

import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import launch_ros.descriptions



def generate_launch_description():
    auv_slam_share = get_package_share_directory('auv_slam')
    auv_slam_prefix = get_package_prefix('auv_slam')
    
    urdf_file = os.path.join(auv_slam_share, 'urdf', 'orca4_description.urdf')
    rviz_config = os.path.join(auv_slam_share, 'rviz', 'urdf_config.rviz')
    bridge_config = os.path.join(auv_slam_share, 'config', 'ign_bridge.yaml')
    thruster_params = os.path.join(auv_slam_share, 'config', 'thruster_params.yaml')
    flare_params = os.path.join(auv_slam_share, 'config', 'flare_params.yaml')
    gate_params = os.path.join(auv_slam_share, 'config', 'gate_params.yaml')
    safety_params = os.path.join(auv_slam_share, 'config', 'safety_params.yaml')
    world_file = os.path.join(auv_slam_share, 'worlds', 'underwater_world_ign.sdf')
    
    # CRITICAL: Correct path to installed script
    flare_order_script = os.path.join(auv_slam_prefix, 'lib', 'auv_slam', 'prompt_flare_order.py')
    
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
    
    # Launch arguments
    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )
    
    declare_spawn_x = DeclareLaunchArgument(
        'spawn_x',
        default_value='-20.0',
        description='X position for robot spawn'
    )
    
    declare_spawn_y = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Y position for robot spawn'
    )
    
    declare_spawn_z = DeclareLaunchArgument(
        'spawn_z',
        default_value='-0.6',
        description='Z position for robot spawn'
    )

    
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
    
    # Spawn Robot Entity (Delayed 2 seconds)
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
                    "-z", LaunchConfiguration('spawn_z'),
                    "-x", LaunchConfiguration('spawn_x'),
                    "-y", LaunchConfiguration('spawn_y'),
                    "-Y", "0.0",
                    "--ros-args", "--log-level", "warn"
                ],
                parameters=[{"use_sim_time": True}],
            )
        ]
    )
    
    # ROS-Gazebo Bridge
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

    
    # Thruster Mapper
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

    
    # Task Coordinator
    task_coordinator = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='auv_slam',
                executable='task_coordinator_node.py',
                name='task_coordinator',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # Flare Detector
    flare_detector = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='auv_slam',
                executable='flare_detector_node.py',
                name='flare_detector',
                output='screen',
                parameters=[flare_params, {'use_sim_time': True}]
            )
        ]
    )
    
    # Flare Navigator
    flare_navigator = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='auv_slam',
                executable='flare_navigator_node.py',
                name='flare_navigator',
                output='screen',
                parameters=[flare_params, {'use_sim_time': True}]
            )
        ]
    )
    
    # Gate Detector
    gate_detector = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='auv_slam',
                executable='gate_detector_node.py',
                name='gate_detector',
                output='screen',
                parameters=[gate_params, {'use_sim_time': True}]
            )
        ]
    )
    
    # Gate Navigator
    gate_navigator = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='auv_slam',
                executable='gate_navigator_node.py',
                name='gate_navigator',
                output='screen',
                parameters=[gate_params, {'use_sim_time': True}]
            )
        ]
    )
    
    # Safety Monitor
    safety_monitor = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='auv_slam',
                executable='safety_monitor_node.py',
                name='safety_monitor',
                output='screen',
                parameters=[safety_params, {'use_sim_time': True}]
            )
        ]
    )

    flare_order_prompt = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', flare_order_script],
                name='flare_order_prompt',
                output='screen',
                shell=False,
                emulate_tty=True,
            )
        ]
    )

    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )

    flare_debug_viewer = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rqt_image_view',
                executable='rqt_image_view',
                name='flare_debug_viewer',
                arguments=['/flare/debug_image'],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    gate_debug_viewer = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rqt_image_view',
                executable='rqt_image_view',
                name='gate_debug_viewer',
                arguments=['/gate/debug_image'],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    
    return LaunchDescription([
        declare_enable_rviz,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,

        robot_state_publisher,
        joint_state_publisher,
        gazebo_process,
        spawn_entity, 
        bridge,      
        thruster_mapper,
        task_coordinator, 
        safety_monitor, 
        
        flare_detector,
        flare_navigator, 
        
        gate_detector,
        gate_navigator,
        flare_order_prompt,

        rviz_node,
        
        flare_debug_viewer,
        gate_debug_viewer,
    ])


if __name__ == '__main__':
    generate_launch_description()