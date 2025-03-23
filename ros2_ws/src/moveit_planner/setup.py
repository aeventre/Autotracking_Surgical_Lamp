from setuptools import setup

package_name = 'moveit_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/moveit_planner.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Motion planner node for the auto-tracking surgical lamp using MoveIt',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'moveit_planner_node = moveit_planner.moveit_planner_node:main',
        ],
    },
)
