from setuptools import setup
import os
from glob import glob

package_name = 'aruco_remote_tracker'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
    ],
    zip_safe=True,
    maintainer='alec',
    maintainer_email='aeventre@buffalo.edu',
    description='Tracks ArUco marker cube on remote to estimate its 3D pose.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_pose_estimator_node = aruco_remote_tracker.marker_pose_estimator_node:main',
        ],
    },
)
