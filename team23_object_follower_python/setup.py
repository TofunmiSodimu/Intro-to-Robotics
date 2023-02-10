#!/usr/bin/env python3
from setuptools import setup

package_name = 'team23_object_follower_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tofunmi Sodimu',
    maintainer_email='tofsodimu@gmail.com',
    description='Object follower',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = team23_object_follower_python.find_object:main',
                'listener = team23_object_follower_python.rotate_robot:main',
        ],
},
)
