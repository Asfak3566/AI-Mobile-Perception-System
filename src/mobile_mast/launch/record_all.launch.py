import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from datetime import datetime

def generate_launch_description():
    # Create a timestamp for the bag filename
    timestamp = datetime.now().strftime('%Y_%m_%d-%H_%M_%S')
    default_bag_name = f'mobile_mast_recording_{timestamp}'

    # Arguments
    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value=default_bag_name,
        description='Name of the rosbag directory'
    )

    storage_path_arg = DeclareLaunchArgument(
        'storage_path',
        default_value=os.path.join(os.environ['HOME'], 'recordings'),
        description='Base path to store recordings'
    )

    # Topics to record
    # We record:
    # 1. Raw Lidar (Orin 1)
    # 2. Raw Images (Orin 1) - Compressed to save space/bandwidth
    # 3. 3D Tracks (Orin 3) - Final output
    # 4. Transforms - Critical for replay
    topics_to_record = [
        '/mast/orin1/lidar/points',
        '/mast/orin1/bullet1/image_raw/compressed',
        '/mast/orin1/bullet2/image_raw/compressed',
        '/mast/orin1/bullet3/image_raw/compressed',
        '/mast/orin1/bullet4/image_raw/compressed',
        '/mast/global/tracks_3d',
        '/tf',
        '/tf_static'
    ]

    # Construct the ros2 bag record command
    # -o specifies the output directory name
    # -s mcap is often more efficient, but standard sqlite3 is fine too. Let's stick to default or specify if needed.
    # We use shell expansion for the command construction in Python
    
    cmd = ['ros2', 'bag', 'record', '-o']
    cmd.append(LaunchConfiguration('storage_path')) # This needs to be combined with bag_name, but ros2 bag -o takes a full path
    
    # Actually, let's simplify. We'll just pass the full path or let the user decide.
    # But to make it robust, let's construct the full path in the command.
    # However, LaunchConfiguration is a substitution, resolved at runtime.
    # ExecuteProcess cmd list expects strings or substitutions.
    
    # Let's try a simpler approach: just record to a directory.
    
    record_cmd = [
        'ros2', 'bag', 'record',
        '--output', [LaunchConfiguration('storage_path'), '/', LaunchConfiguration('bag_name')],
    ] + topics_to_record

    recorder = ExecuteProcess(
        cmd=record_cmd,
        output='screen'
    )

    return LaunchDescription([
        bag_name_arg,
        storage_path_arg,
        recorder
    ])
