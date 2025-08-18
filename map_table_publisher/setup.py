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
        ],
    },
)
