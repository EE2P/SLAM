import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'follow_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stalker',
    maintainer_email='haoxiaosa58@gmail.com',
    description='Metric person-follow bridge: lock one tracked person -> /cmd_vel Twist',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'follow_bridge_node = follow_bridge.follow_bridge_node:main',
        ],
    },
)
