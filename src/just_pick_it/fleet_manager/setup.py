import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'fleet_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='suwoo3131',
    maintainer_email='suwoo3131@gmail.com',
    description='Fleet Manager: Traffic Manager and Task Manager for just_pick_it AMR system',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fleet_manager_node = fleet_manager.fleet_manager_node:main',
        ],
    },
)
