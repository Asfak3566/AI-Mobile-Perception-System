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
    # Note: 'use_lidar_clock': True assumes the LiDAR itself is PTP synchronized.
    # If the LiDAR is NOT PTP synchronized but the Orin is, set this to False
    # to use the Orin's system time (which is PTP synced) for stamping.
    lidar_params = [{
        'frame_id': 'rslidar',
        'lidar_type': 'RS16',
        'device_ip': '192.168.6.60',
        'msop_port': 1660,
        'difop_port': 2660,
        'use_lidar_clock': True,
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
        bullet_name = ns.split('/')[-1]
        intrinsics_yaml = os.path.join(
            mobile_mast_share, 'config', 'camera', f'{bullet_name}_intrinsics.yaml'
        )

        nodes.append(
            GroupAction([
                # Driver
                Node(
                    package='mobile_mast',
                    executable='axis_camera_ros',
                    name='axis_camera_ros',
                    namespace=ns,
                    output='screen',
                    parameters=camera_common_params + [{'ip': ip}],
                ),
                # Camera Info
                Node(
                    package='mobile_mast',
                    executable='camerainfo_publisher',
                    name='camerainfo_publisher',
                    namespace=ns,
                    output='screen',
                    parameters=[{
                        'camera_info_yaml': intrinsics_yaml,
                        'camera_info_topic': 'camera_info',
                        'frame_id': bullet_name,
                        'publish_rate_hz': 10.0,
                    }],
                ),
            ])
        )

    # ───── 3) Recording ──────────────────────────────────
    record_arg = DeclareLaunchArgument(
        'record',
        default_value='False',
        description='Record ROS 2 bag (True/False)'
    )

    # Topics to record for Orin 1 (Sensors)
    topics_to_record = [
        '/mast/orin1/lidar/points',
        '/mast/orin1/lidar/points_filtered',
        '/mast/orin1/lidar/foreground',
        
        '/mast/orin1/bullet1/image_raw',
        '/mast/orin1/bullet1/image_raw/compressed',
        '/mast/orin1/bullet1/camera_info',
        
        '/mast/orin1/bullet2/image_raw',
        '/mast/orin1/bullet2/image_raw/compressed',
        '/mast/orin1/bullet2/camera_info',
        
        '/mast/orin1/bullet3/image_raw',
        '/mast/orin1/bullet3/image_raw/compressed',
        '/mast/orin1/bullet3/camera_info',
        
        '/mast/orin1/bullet4/image_raw',
        '/mast/orin1/bullet4/image_raw/compressed',
        '/mast/orin1/bullet4/camera_info',
        
        '/tf', 
        '/tf_static'
    ]

    from launch.actions import ExecuteProcess
    from launch.conditions import IfCondition

    record_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record'] + topics_to_record,
        output='screen',
        condition=IfCondition(LaunchConfiguration('record'))
    )

    nodes.append(record_process)

    return LaunchDescription([replay_arg, record_arg] + nodes)

