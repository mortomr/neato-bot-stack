from setuptools import find_packages, setup

package_name = 'neato_bot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/neato_bot_control']),
        ('share/neato_bot_control', ['package.xml']),
        ('share/neato_bot_control/launch', [
            'neato_bot_control/launch/motor_bridge.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
entry_points={
    'console_scripts': [
        'ros2_motor_bridge = neato_bot_control.ros2_motor_bridge:main',
        'cmd_vel_listener = neato_bot_control.cmd_vel_listener:main'
    ],
},

)
