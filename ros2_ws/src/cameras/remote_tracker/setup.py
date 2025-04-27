from setuptools import find_packages, setup

package_name = 'remote_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/remote_tracker.launch.py']),
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
            'remote_tracker_node = remote_tracker.remote_tracker_node:main'
        ],
    },
)
