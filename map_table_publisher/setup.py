from setuptools import setup
import setuptools

setup(
    name='table_obstacle_layer',
    version='0.0.1',
    packages=setuptools.find_packages(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='your_email@example.com',
    description='Table obstacle detection node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'table_obstacle_node = table_obstacle_layer.table_obstacle_node:main',
        ],
    },
)
