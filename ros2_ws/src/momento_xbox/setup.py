from setuptools import find_packages, setup

package_name = 'momento_xbox'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='han',
    maintainer_email='han.li124@imperial.ac.uk',
    description='Xbox controller mode and command adapter for Momento',
    license='MIT',
    entry_points={
        'console_scripts': [
            'xbox_mode_node = momento_xbox.xbox_mode_node:main',
            'joy_mapping_probe = momento_xbox.joy_mapping_probe_node:main',
        ],
    },
)
