from setuptools import find_packages, setup

package_name = 'camera_estimation_2d'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/hand_position.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'cv_bridge',
        'sensor_msgs',
        'geometry_msgs',
        'mediapipe',
        'opencv-python',
    ],

    zip_safe=True,
    maintainer='alec',
    maintainer_email='aeventre@buffalo.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_position_publisher = camera_estimation_2d.hand_position_publisher:main',
        ],
    },
)
