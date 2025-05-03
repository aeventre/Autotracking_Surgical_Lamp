from setuptools import setup
import os
from glob import glob

package_name = 'camera_calibration'

setup(
    name=package_name,
<<<<<<< HEAD
    version='0.0.1',
=======
    version='0.0.0',
>>>>>>> mcu1
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
<<<<<<< HEAD
    install_requires=[
        'setuptools',
        'opencv-python',
    ],
    zip_safe=True,
    maintainer='alec',
    maintainer_email='aeventre@buffalo.edu',
    description='Camera calibration node using OpenCV ArUco markers.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_calibration_node = camera_calibration.camera_calibration_node:main'
=======
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alec',
    maintainer_email='aeventre@buffalo.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_calibrator_node = camera_calibration.camera_calibrator_node:main'
>>>>>>> mcu1
        ],
    },
)

