from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the directory of the package
    package_dir = get_package_share_directory('obstacle_cloud_to_scan')

    return LaunchDescription([
        Node(
            package='obstacle_cloud_to_scan',
            executable='obstacle_cloud_to_scan',
            name='obstacle_cloud_to_scan_node',
            output='screen',
            # arguments=['--ros-args', '--log-level', 'debug'],
            parameters=[{
                'input_topic': '/livox/lidar',
                'output_topic': '/filtered_point_cloud',
                'laser_scan_topic': '/scan',
                'voxel_leaf_size': 0.1,
                'max_distance': 10.0,
                'min_distance': 0.1,
                'robot_box_size': [0.9, 0.8, 1.0],
                'robot_box_position': [0.0, 0.0, 0.0],
                'max_slope_angle': 20.0,
                'use_gpu': False
            }],
            remappings=[
                ('input_topic', '/livox_cloud_in'),
                ('output_topic', '/filtered_point_cloud'),
                ('laser_scan_topic', '/scan')
            ]
        )
    ])

