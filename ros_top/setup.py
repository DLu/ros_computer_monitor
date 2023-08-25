#!/usr/bin/env python

from setuptools import setup

package_name = 'ros_top'

setup(
    name=package_name,
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/ros_top']),
    ],
    packages=[package_name],
    package_dir={'': 'src'},
    version='0.4.1',
    description='For monitoring ROS processes.',
    license='BSD 3-clause',
    maintainer='Dan Lazewatsky',
    maintainer_email='dan@lazewatsky.com',
    entry_points={'console_scripts': ['ros_top = ros_top.ros_top:main', 'viz = ros_top.viz:main', 'bag_viz = ros_top.bag_viz:main']},
)
