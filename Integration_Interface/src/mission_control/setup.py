from setuptools import setup

package_name = 'mission_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zesong',
    maintainer_email='your.email@northeastern.edu',
    description='Mission control system for TurtleBot 4',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_machine = mission_control.state_machine:main',
        ],
    },
)