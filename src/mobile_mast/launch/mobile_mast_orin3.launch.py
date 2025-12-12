import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    replay_arg = DeclareLaunchArgument(
        'replay',
        default_value='False',
        description='Replay mode (kept for consistency, not used much here)'
    )

    nodes = []

    mobile_mast_share = get_package_share_directory('mobile_mast')

    camera_config_dir = os.path.join(
        mobile_mast_share, 'config', 'camera'
    )

    fusion_config_path = os.path.join(
        mobile_mast_share, 'config', 'lidar', 'global_fusion.yaml'
    )

    # ───── 1) EKF trackers for all 4 bullets ─────────────
    bullet_cams = [
        'bullet1',
        'bullet2',
        'bullet3',
        'bullet4',
    ]

    for bullet_name in bullet_cams:
        tracker_cfg = os.path.join(
            camera_config_dir, f'tracker_{bullet_name}.yaml'
        )

        nodes.append(
            GroupAction([
                Node(
                    package='mobile_mast',
                    executable='camera_tracker',
                    name=f'camera_tracker_{bullet_name}',
                    namespace=f'mast/orin1/{bullet_name}',
                    output='screen',
                    parameters=[{
                        'tracker_config_file': tracker_cfg,
                    }],
                )
            ])
        )

    # ───── 2) Global fusion node ────────────────────────
    nodes.append(
        GroupAction([
            Node(
                package='mobile_mast',
                executable='global_fusion_node',
                name='global_fusion_node',
                namespace='',
                output='screen',
                parameters=[{
                    'fusion_config_file': fusion_config_path,
                }],
            )
        ])
    )

     # ───── 3) Markers for RViz (global tracks → MarkerArray) ─────
    nodes.append(
        GroupAction([
            Node(
                package='mobile_mast',
                executable='tracks_to_markers_node',
                name='tracks_to_markers_node',
                namespace='',
                output='screen',
                parameters=[{
                    'tracks_topic': '/mast/global/tracks_3d',
                    'markers_topic': '/mast/global/tracks_markers',
                }],
            )
        ])
    )

    return LaunchDescription([replay_arg] + nodes)

