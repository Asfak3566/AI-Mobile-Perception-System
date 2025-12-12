import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    replay_arg = DeclareLaunchArgument(
        'replay',
        default_value='False',
        description='Replay mode (True/False)'
    )

    global_params = [{
        'replay': LaunchConfiguration('replay'),
    }]

    nodes = []

    mobile_mast_share = get_package_share_directory('mobile_mast')
    lidar_cluster_params_path = os.path.join(
        mobile_mast_share, 'config', 'lidar', 'lidar_cluster_params.yaml'
    )

    # ───── 1) RoboSense LiDAR driver ─────────────────────
    lidar_params = [{
        'frame_id': 'rslidar',
        'lidar_type': 'RS16',
        'device_ip': '192.168.6.60',
        'msop_port': 1660,
        'difop_port': 2660,
    }]

    nodes.append(
        GroupAction([
            Node(
                package='rslidar_sdk',
                executable='rslidar_sdk_node',
                name='rslidar_sdk_node',
                namespace='mast/orin1/lidar',
                output='screen',
                parameters=global_params + lidar_params,
            )
        ])
    )

    # ───── 1.b) LiDAR preprocessing ──────────────────────
    nodes.append(
        GroupAction([
            Node(
                package='mobile_mast',
                executable='rslidar_preprocess',
                name='rslidar_preprocess',
                namespace='',
                output='screen',
                parameters=[{
                    'input_topic': '/mast/orin1/lidar/points',
                    'output_topic': '/mast/orin1/lidar/points_filtered',
                    'min_range': 1.0,
                    'max_range': 150.0,
                    'enable_ground_removal': True,
                    'ground_z_min': -0.3,
                    'ground_z_max': 0.3,
                    'enable_roi_filter': False,
                    'roi_min_x': -50.0,
                    'roi_max_x':  50.0,
                    'roi_min_y': -50.0,
                    'roi_max_y':  50.0,
                    'roi_min_z': -3.0,
                    'roi_max_z':  5.0,
                }],
            )
        ])
    )

    # ───── 1.c) Background Subtraction ───────────────────
    background_pcd_path = os.path.join(
        mobile_mast_share, 'config', 'lidar', 'background_ascii.pcd'
    )
    
    nodes.append(
        GroupAction([
            Node(
                package='mobile_mast',
                executable='lidar_background_subtraction',
                name='lidar_background_subtraction',
                namespace='',
                output='screen',
                parameters=[{
                    'input_topic': '/mast/orin1/lidar/points_filtered',
                    'output_topic': '/mast/orin1/lidar/foreground',
                    'background_pcd_file': background_pcd_path,
                    'voxel_size': 0.3,
                }],
            )
        ])
    )

    # ───── 1.d) LiDAR clustering ─────────────────────────
    nodes.append(
        GroupAction([
            Node(
                package='mobile_mast',
                executable='lidar_cluster_node',
                name='lidar_cluster_node',
                namespace='',
                output='screen',
                parameters=[lidar_cluster_params_path, {
                    'input_topic': '/mast/orin1/lidar/foreground',  # Use foreground now
                }],
            )
        ])
    )

    # ───── 2) Axis cameras (4x) – image publishers only ──
    camera_common_params = [{
        'username': 'root',
        'password': 'Inf_MobileMast_1',
        'replay': LaunchConfiguration('replay'),
    }]

    bullet_cams = [
        ('mast/orin1/bullet1', '192.168.6.41'),
        ('mast/orin1/bullet2', '192.168.6.42'),
        ('mast/orin1/bullet3', '192.168.6.43'),
        ('mast/orin1/bullet4', '192.168.6.44'),
    ]

    for ns, ip in bullet_cams:
        nodes.append(
            GroupAction([
                Node(
                    package='mobile_mast',
                    executable='axis_camera_ros',
                    name='axis_camera_ros',
                    namespace=ns,
                    output='screen',
                    parameters=camera_common_params + [{'ip': ip}],
                )
            ])
        )

    return LaunchDescription([replay_arg] + nodes)

