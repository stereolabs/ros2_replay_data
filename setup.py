from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_replay_data'

def package_files(directory):
    paths = []
    for (path, _, filenames) in os.walk(directory):
        for filename in filenames:
            filepath = os.path.join(path, filename)
            install_path = os.path.join('share', package_name, path)
            paths.append((install_path, [filepath]))
    return paths

setup(
    name=package_name,
    version='1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ] + package_files('examples') + package_files('rviz') + package_files('urdf'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stereolabs',
    maintainer_email='support@stereolabs.com',
    description='Package allowing to sync data between a SVO and a ROS2 bag',
    license='Stereolabs',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'sync_node = ros2_replay_data.sync_node:main',
        'dual_camera_sync_node = ros2_replay_data.dual_camera_sync_node:main',
        'control_svo_node = ros2_replay_data.control_svo_node:main'
        ],
    },
)
