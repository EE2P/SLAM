import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rs_nav_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='han',
    maintainer_email='han.li124@imperial.ac.uk',
    description='RealSense camera-only RTAB-Map SLAM + Nav2 navigation fallback',
    license='MIT',
)
