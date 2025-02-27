from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'kinematics_2d'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alec',
    maintainer_email='aeventre@buffalo.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinematics_2d = 2d_kinematics.ik_node_2d:main',
        ],
    },
        data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
)
