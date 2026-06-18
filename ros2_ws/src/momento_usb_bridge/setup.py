import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'momento_usb_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='han',
    maintainer_email='han.li124@imperial.ac.uk',
    description='100 Hz USB CDC bridge for Momento balance commands',
    license='MIT',
    entry_points={
        'console_scripts': [
            'usb_bridge_node = momento_usb_bridge.usb_bridge_node:main',
        ],
    },
)
