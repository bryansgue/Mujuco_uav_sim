from setuptools import setup
import os
from glob import glob

package_name = 'drone_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('bin', glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'teleop = drone_teleop.teleop:main',
            'wind_publisher = drone_teleop.wind_publisher:main',
        ],
    },
)
