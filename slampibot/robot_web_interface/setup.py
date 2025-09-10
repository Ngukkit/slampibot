from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_web_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'templates'), glob(os.path.join('robot_web_interface', 'templates', '*.html'))),
    ],
    install_requires=['setuptools', 'flask'],
    zip_safe=True,
    maintainer='kkit',
    maintainer_email='ppakdone@gmail.com',
    description='Web interface for robot control',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_app = robot_web_interface.app:main',
        ],
    },
)