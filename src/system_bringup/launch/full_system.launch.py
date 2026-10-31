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

    # Multi-Layer Personnel Security Stack Nodes
    personnel_tag_node = Node(
        package='personnel_security',
        executable='personnel_tag_node',
        output='screen'
    )
    credential_authentication_node = Node(
        package='personnel_security',
        executable='credential_authentication_node',
        output='screen'
    )
    face_verification_node = Node(
        package='personnel_security',
        executable='face_verification_node',
        output='screen'
    )
    identity_correlation_node = Node(
        package='personnel_security',
        executable='identity_correlation_node',
        output='screen'
    )
    activity_assessment_node = Node(
        package='personnel_security',
        executable='activity_assessment_node',
        output='screen'
    )
    personnel_event_logger = Node(
        package='personnel_security',
        executable='personnel_event_logger',
        output='screen'
    )

    return LaunchDescription([
        simulation_include,
        battery_node,
        mission_node,
        perception_node,
        event_node,
        orchestrator_node,
        personnel_tag_node,
        credential_authentication_node,
        face_verification_node,
        identity_correlation_node,
        activity_assessment_node,
        personnel_event_logger
    ])
