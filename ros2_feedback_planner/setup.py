import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ros2_feedback_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juan',
    maintainer_email='juan97pena@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'feedback_node = ros2_feedback_planner.feedback.feedback_server:main',
            'planner_node = ros2_feedback_planner.planning.simple_planner:main',
            'metrics_manager_node = ros2_feedback_planner.metrics_manager:main',
            'manipulator_simulator = ros2_feedback_planner.manipulator_simulator:main',
        ],
    },
)
