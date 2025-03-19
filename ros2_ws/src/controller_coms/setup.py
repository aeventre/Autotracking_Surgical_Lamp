from setuptools import find_packages, setup

package_name = 'controller_coms'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='alec',
    maintainer_email='aeventre@buffalo.edu',
    description='ROS2 package for communicating with the MCUs of the surgical lamp system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coms_node = controller_coms.coms_node:main',
        ],
    },
)