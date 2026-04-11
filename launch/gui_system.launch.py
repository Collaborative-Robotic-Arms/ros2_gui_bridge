#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # ---------------------------------------------------------
    # 1. Read Launch Arguments
    # ---------------------------------------------------------
    mode = LaunchConfiguration('mode').perform(context)
    is_sim = (mode == 'sim')

    nodes_to_start = []

    # ---------------------------------------------------------
    # 2. COMMON NODES
    # ---------------------------------------------------------
    
    # Firestore Bridge (Runs identically in both modes)
    nodes_to_start.append(
        Node(
            package='ros2_gui_bridge',
            executable='firestore_bridge.py',
            name='firestore_bridge_node',
            output='screen'
        )
    )

    # Brick Processor Node
    nodes_to_start.append(
        Node(
            package='ros2_gui_bridge',
            executable='brick_processor2.py',
            name='brick_processor',
            output='screen',
            parameters=[{'use_sim': is_sim}]
        )
    )

    # Assembly Plan / Allocator Node
    nodes_to_start.append(
        Node(
            package='ros2_gui_bridge',
            executable='assembly_plan.py',
            name='assembly_allocator',
            output='screen',
            parameters=[{'use_sim': is_sim}]
        )
    )

    return nodes_to_start

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='sim',
            choices=['sim', 'real'],
            description='System mode: "sim" (default) or "real".'
        ),
        OpaqueFunction(function=launch_setup)
    ])