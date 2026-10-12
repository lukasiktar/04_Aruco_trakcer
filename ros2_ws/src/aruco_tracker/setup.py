import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'aruco_tracker'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zvonimir',
    maintainer_email='zvonimir@todo.todo',
    description='Detects ArUco markers, draws their axes and centres on the image and outputs cmd_vel.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_tracker_node = aruco_tracker.aruco_tracker_node:main',
        ],
    },
)
