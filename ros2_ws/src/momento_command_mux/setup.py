from setuptools import find_packages, setup

package_name = 'momento_command_mux'

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
    description='Command arbitration for Momento manual, auto and assist control',
    license='MIT',
    entry_points={
        'console_scripts': [
            'command_mux_node = momento_command_mux.command_mux_node:main',
        ],
    },
)
