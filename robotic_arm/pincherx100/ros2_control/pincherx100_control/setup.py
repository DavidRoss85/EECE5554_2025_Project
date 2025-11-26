from setuptools import setup
from glob import glob
import os

package_name = 'pincherx100_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PincherX100 User',
    maintainer_email='user@example.com',
    description='ROS2 control package for PincherX100 robotic arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_controller = pincherx100_control.arm_controller_node:main',
            'calibrate = pincherx100_control.calibrate_arm:main',
            'test_joints = pincherx100_control.test_joints:main',
        ],
    },
)



