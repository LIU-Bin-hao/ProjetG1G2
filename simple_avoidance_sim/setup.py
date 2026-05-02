from setuptools import setup
import os
from glob import glob

package_name = 'simple_avoidance_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LIU Binhao',
    maintainer_email='binhao.liu@centrale.centralelille.fr',
    description='Simple Gazebo obstacle avoidance simulation package for ROS 2 Humble',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'obstacle_avoidance_node = simple_avoidance_sim.obstacle_avoidance_node:main',
        'ackermann_controller_node = simple_avoidance_sim.ackermann_controller_node:main',
        'goal_navigation_node = simple_avoidance_sim.goal_navigation_node:main',
        'road_detector = simple_avoidance_sim.road_detector:main',
    	],
    },
)
