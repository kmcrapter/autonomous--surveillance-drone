"""ROS 2 Launch file bringing up complete Digital Twin system stack."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    ws_dir = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws"
    sim_launch = os.path.join(ws_dir, "src/system_bringup/launch/simulation.launch.py")

    # Simulation & Bridge
    simulation_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch)
    )

    # Battery Manager Node
    battery_node = Node(
        package='battery_manager',
        executable='battery_node',
        output='screen'
    )

    # Mission Manager Node
    mission_node = Node(
        package='mission_manager',
        executable='mission_node',
        output='screen'
    )

    # Perception Pipeline Node
    perception_node = Node(
        package='perception',
        executable='perception_node',
        output='screen'
    )

    # Event Manager Node
    event_node = Node(
        package='event_manager',
        executable='event_node',
        output='screen'
    )

    # System Orchestrator Node
    orchestrator_node = Node(
        package='system_bringup',
        executable='orchestrator',
        output='screen'
    )

    return LaunchDescription([
        simulation_include,
        battery_node,
        mission_node,
        perception_node,
        event_node,
        orchestrator_node
    ])
