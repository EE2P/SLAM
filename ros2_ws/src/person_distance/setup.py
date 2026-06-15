import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'person_distance'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'foxglove'), glob('foxglove/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='test',
    maintainer_email='han.li124@imperial.ac.uk',
    description='YOLO person segmentation + depth-based average distance',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'person_distance_node = person_distance.person_distance_node:main',
            'perception_benchmark = person_distance.benchmark_node:main',
        ],
    },
)
