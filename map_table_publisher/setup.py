from setuptools import setup
import os
from glob import glob

package_name = 'map_table_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='your_email@example.com',
    description='Table obstacle detection node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tag_publisher_node = map_table_publisher.tag_publisher:main',
            'polygon_publisher_node = map_table_publisher.polygon_publisher:main',
            'ceiling_camera_processor = map_table_publisher.ceiling_camera_processor:main',
            'improved_camera_processor = map_table_publisher.improved_camera_processor:main',
            'calibrated_camera_processor = map_table_publisher.calibrated_camera_processor:main',
            'camera_calibration_node = map_table_publisher.camera_calibration_node:main',
            'debug_visualizer = map_table_publisher.debug_visualizer:main',
            'rectified_camera_info_publisher = map_table_publisher.rectified_camera_info_publisher:main',
            'raw_ceiling_camera_processor = map_table_publisher.raw_ceiling_camera_processor:main',
            'apriltag_persistence_node = map_table_publisher.apriltag_persistence_node:main',
            'apriltag_tracker_node = map_table_publisher.apriltag_tracker_node:main',
            'image_preprocessor_node = map_table_publisher.image_preprocessor_node:main',
        ],
    },
)
