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

    camera_config_dir = os.path.join(
        mobile_mast_share, 'config', 'camera'
    )

    yolo_weights_path = '/home/orin1/Downloads/mobile_mast_ws/yolo11n.pt'

    # ───── 1) YOLO11 + 3D fusion for each bullet ─────────
    bullet_cams = [
        'bullet1',
        'bullet2',
        'bullet3',
        'bullet4',
    ]

    for bullet_name in bullet_cams:
        ns = f'mast/orin1/{bullet_name}'
        intrinsics_yaml = os.path.join(camera_config_dir, f'{bullet_name}_intrinsics.yaml')

        nodes.append(
            GroupAction([
                # 1. Decompress: image_raw/compressed -> image_raw
                Node(
                    package='image_transport',
                    executable='republish',
                    name=f'republish_{bullet_name}',
                    namespace=ns,
                    arguments=['compressed', 'raw'],
                    remappings=[
                        ('in/compressed', 'image_raw/compressed'),
                        ('out', 'image_raw'),
                    ],
                    output='screen',
                ),

                # 2. Publish CameraInfo
                Node(
                    package='mobile_mast',
                    executable='camerainfo_publisher',
                    name=f'camerainfo_{bullet_name}',
                    namespace=ns,
                    output='screen',
                    parameters=[{
                        'camera_info_yaml': intrinsics_yaml,
                        'camera_info_topic': 'camera_info',
                        'frame_id': bullet_name,
                        'publish_rate_hz': 10.0,
                    }],
                ),

                # 3. Undistort: image_raw + camera_info -> image_rect
                Node(
                    package='mobile_mast',
                    executable='vpi_undistort',
                    name=f'vpi_undistort_{bullet_name}',
                    namespace=ns,
                    output='screen',
                    parameters=[{
                        'image_topic': 'image_raw',
                        'camera_info_topic': 'camera_info',
                        'output_topic': 'image_rect',
                        'use_vpi': True,
                    }],
                ),

                # 4. YOLO: image_rect -> detections_2d
                Node(
                    package='mobile_mast',
                    executable='yolo11_detector',
                    name=f'yolo11_detector_{bullet_name}',
                    namespace=ns,
                    output='screen',
                    parameters=[{
                        'weights': yolo_weights_path,
                        'input_topic': 'image_rect',
                        'conf_threshold': 0.35,
                    }],
                ),

                # 5. 3D Fusion: detections_2d + lidar -> detections_3d
                Node(
                    package='mobile_mast',
                    executable='camera_image_to_3d',
                    name=f'camera_image_to_3d_{bullet_name}',
                    namespace=ns,
                    output='screen',
                    parameters=[{
                        'detections_topic': 'detections_2d',  # /mast/orin1/bulletX/detections_2d
                        'lidar_topic': '/mast/orin1/lidar/foreground',
                        'output_topic': 'detections_3d',      # /mast/orin1/bulletX/detections_3d
                        'intrinsics_file': intrinsics_yaml,
                        'extrinsics_file': os.path.join(camera_config_dir, f'lidar_to_{bullet_name}.yaml'),
                        'min_points_per_box': 20,
                    }],
                ),
            ])
        )

    # ───── 2) Recording ──────────────────────────────────
    record_arg = DeclareLaunchArgument(
        'record',
        default_value='False',
        description='Record ROS 2 bag (True/False)'
    )

    # Topics to record for Orin 2 (Detections)
    topics_to_record = [
        '/mast/orin1/bullet1/detections_2d',
        '/mast/orin1/bullet1/detections_3d',
        '/mast/orin1/bullet2/detections_2d',
        '/mast/orin1/bullet2/detections_3d',
        '/mast/orin1/bullet3/detections_2d',
        '/mast/orin1/bullet3/detections_3d',
        '/mast/orin1/bullet4/detections_2d',
        '/mast/orin1/bullet4/detections_3d',
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

