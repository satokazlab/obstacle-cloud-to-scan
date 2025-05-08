from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the directory of the package (optional, if needed)
    # package_dir = get_package_share_directory('obstacle_cloud_to_scan')

    return LaunchDescription([
    
        Node(
            package='obstacle_cloud_to_scan',
            executable='obstacle_cloud_to_scan',
            name='obstacle_cloud_to_scan_node',
            # arguments=['--ros-args', '--log-level', 'debug'],
            output='screen',
            parameters=[{
                'input_topic': '/livox/lidar',
                #'output_topic': '/filtered_point_cloud',
                'output_topic': '/cloud_in', # pointcloud_to_laserscanに入力
                'voxel_leaf_size': 0.1,
                'robot_box_size': [0.9, 0.8, 1.0],
                'robot_box_position': [0.0, 0.0, 0.0],
                'max_slope_angle': 25.0,
                'use_gpu': False
            }],
            remappings=[
                #('input_topic', '/livox_cloud_in'),
                #('output_topic', '/filtered_point_cloud'),
                ('output_topic', '/cloud_in'),
            ]
        ),
        
        Node(
            package='pointcloud_to_laserscan', 
            executable='pointcloud_to_laserscan_node',
            # arguments=['--ros-args', '--log-level', 'debug'],
            name='pointcloud_to_laserscan_node',
            output='screen',
            remappings=[
                ('input_topic', '/cloud_in'),  # 確認が必要
                ('output_topic', '/scan')  # 通常のトピック名に合わせる
            ],
            parameters=[{
                'target_frame': 'livox_frame',  # 空文字列から修正
                'transform_tolerance': 0.01,
                'min_height': -1.0,
                'max_height': 2.0,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,   # M_PI/2
                'angle_increment': 0.0174,  # M_PI/360.0
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 40.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }]
        )
    ])
