from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'command_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
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
            'command_manager_node = command_manager.command_manager_node:main',
            'gui_node = command_manager.gui_node:main',
        ],
    },
)
