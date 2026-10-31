"""ROS 2 Launch file for Gazebo Harmonic 3D World and Ros-Gz Bridges."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ws_dir = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws"
    world_sdf = os.path.join(ws_dir, "src/surveillance_world/worlds/border_surveillance.sdf")
    models_dir = os.path.join(ws_dir, "src/drone_description/models")

    # Set GZ Resource Paths
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{models_dir}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    )

    # Launch Gazebo Harmonic
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_sdf],
        output='screen'
    )

    # Ros-Gz Parameter Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/drone/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/drone/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/drone/camera/forward/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/drone/camera/downward/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/drone/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/drone/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/surveillance_drone/joint/rotor_0_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/surveillance_drone/joint/rotor_1_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/surveillance_drone/joint/rotor_2_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/surveillance_drone/joint/rotor_3_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double'
        ],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        bridge
    ])
