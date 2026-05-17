import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'just_pick_it_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ane',
    maintainer_email='jjeongpark@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'capture_image = just_pick_it_perception.capture_image:main',
            'camera_calibrator = just_pick_it_perception.camera_calibrator:main',
            'apriltag_pose_estimator = just_pick_it_perception.apriltag_pose_estimator:main',
        ],
    },
)
