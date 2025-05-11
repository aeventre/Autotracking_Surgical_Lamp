from setuptools import setup
import os
from glob import glob

package_name = 'camera_calibration'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Ensure launch files are installed
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'opencv-python'],
    zip_safe=True,
    maintainer='alec',
    maintainer_email='aeventre@buffalo.edu',
    description='Camera calibration node using OpenCV ArUco markers.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_calibration_node = camera_calibration.camera_calibration_node:main'
        ],
    },
    include_package_data=True,  # <-- ADD THIS LINE
)


