import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mobile_mast'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install config files recursively
        *[
            (os.path.join('share', package_name, root),
             [os.path.join(root, file) for file in files])
            for root, folders, files in os.walk('config')
        ],
        
        ('share/' + package_name + '/launch',
         glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz',
         glob('rviz/*.rviz')),
    ],
    zip_safe=True,
    maintainer='ashfaq',
    maintainer_email='ashfaq@todo.todo',
    description='Mobile Mast Sensor Fusion Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'axis_camera_ros = mobile_mast.axis_camera_ros:main',
            'yolo11_detector = mobile_mast.yolo11_detector:main',
            'rslidar_preprocess = mobile_mast.rslidar_preprocess:main',
            'lidar_background_subtraction = mobile_mast.lidar_background_subtraction:main',
            'lidar_cluster_node = mobile_mast.lidar_cluster_node:main',
            'camera_image_to_3d = mobile_mast.camera_image_to_3d:main',
            'camera_tracker = mobile_mast.camera_tracker:main',
            'global_fusion_node = mobile_mast.global_fusion_node:main',
            'tracks_to_markers_node = mobile_mast.tracks_to_markers_node:main',
            'convert_calibration = mobile_mast.convert_calibration:main',
            'qt_calibrator = mobile_mast.qt_calibrator:main',
            'camerainfo_publisher = mobile_mast.camerainfo_publisher_node:main',
            'vpi_undistort = mobile_mast.vpi_undistort_node:main',


        ],
    },
)


