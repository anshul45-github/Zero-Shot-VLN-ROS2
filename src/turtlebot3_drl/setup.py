import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'turtlebot3_drl'

setup(
    name=package_name,
    version='2.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author=['Gilbert', 'Ryan Shim'],
    author_email=['kkjong@robotis.com', 'jhshim@robotis.com'],
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    keywords=['ROS', 'ROS2', 'DRL', 'navigation'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='DRL autonomous navigation for TurtleBot3 (Gz Harmonic port)',
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'environment = turtlebot3_drl.drl_environment.drl_environment:main',
            'real_environment = turtlebot3_drl.drl_environment.drl_environment_real:main',
            'gazebo_goals = turtlebot3_drl.drl_gazebo.drl_gazebo:main',
            'train_agent = turtlebot3_drl.drl_agent.drl_agent:main_train',
            'test_agent = turtlebot3_drl.drl_agent.drl_agent:main_test',
            'real_agent = turtlebot3_drl.drl_agent.drl_agent:main_real',
            'obstacle_mover = turtlebot3_drl.obstacle_mover:main',
        ],
    },
)
