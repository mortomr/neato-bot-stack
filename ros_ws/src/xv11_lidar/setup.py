from setuptools import setup
import os
from glob import glob

package_name = 'xv11_lidar'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='you@example.com',
    description='ROS 2 node for XV11 LiDAR',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xv11_lidar = xv11_lidar.lidar_node:main',
        ],
    },

)
