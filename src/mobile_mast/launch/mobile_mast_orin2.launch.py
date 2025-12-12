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

    yolo_weights_path = 'yolo11n.engine'

    # ───── 1) YOLO11 + 3D fusion for each bullet ─────────
    bullet_cams = [
        'bullet1',
        'bullet2',
        'bullet3',
        'bullet4',
    ]

    for bullet_name in bullet_cams:
        ns = f'mast/orin1/{bullet_name}'

        nodes.append(
            GroupAction([
                # YOLO per camera
                Node(
                    package='mobile_mast',
                    executable='yolo11_detector',
                    name=f'yolo11_detector_{bullet_name}',
                    namespace=ns,
                    output='screen',
                    parameters=[{
                        'weights': yolo_weights_path,
                        'input_topic': 'image_raw/compressed',
                        'conf_threshold': 0.35,
                    }],
                ),
                # camera + LiDAR → 3D detections
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
                        'intrinsics_file': os.path.join(camera_config_dir, f'{bullet_name}_intrinsics.yaml'),
                        'extrinsics_file': os.path.join(camera_config_dir, f'lidar_to_{bullet_name}.yaml'),
                        'min_points_per_box': 20,
                    }],
                ),
            ])
        )

    return LaunchDescription([replay_arg] + nodes)

