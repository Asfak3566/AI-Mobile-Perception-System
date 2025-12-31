import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    mobile_mast_share = get_package_share_directory('mobile_mast')
    
    replay_arg = DeclareLaunchArgument(
        'replay',
        default_value='False',
        description='Replay mode (True/False)'
    )

    # 1. Launch Orin 1 (LiDAR + Camera Drivers)
    orin1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mobile_mast_share, 'launch', 'mobile_mast_orin1.launch.py')
        ),
        launch_arguments={
            'replay': LaunchConfiguration('replay'),
            'record': LaunchConfiguration('record')
        }.items()
    )

    # 2. Launch Orin 2 (YOLO + 3D Projection)
    orin2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mobile_mast_share, 'launch', 'mobile_mast_orin2.launch.py')
        ),
        launch_arguments={
            'replay': LaunchConfiguration('replay'),
            'record': LaunchConfiguration('record')
        }.items()
    )

    # 3. Launch Orin 3 (Tracking + Global Fusion)
    orin3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mobile_mast_share, 'launch', 'mobile_mast_orin3.launch.py')
        ),
        launch_arguments={
            'replay': LaunchConfiguration('replay'),
            'record': LaunchConfiguration('record')
        }.items()
    )

    return LaunchDescription([
        replay_arg,
        DeclareLaunchArgument(
            'record',
            default_value='False',
            description='Record ROS 2 bag (True/False)'
        ),
        orin1_launch,
        orin2_launch,
        orin3_launch
    ])
