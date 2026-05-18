import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'just_pick_it_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch', 'vision'),
            glob('launch/vision/*.xml') + glob('launch/vision/*.py')),
        (os.path.join('share', package_name, 'launch', 'inspection'),
            glob('launch/inspection/*.xml') + glob('launch/inspection/*.py')),
        (os.path.join('share', package_name, 'launch', 'sorting'),
            glob('launch/sorting/*.xml') + glob('launch/sorting/*.py')),
        (os.path.join('share', package_name, 'launch', 'amr_1'),
            glob('launch/amr_1/*.xml') + glob('launch/amr_1/*.py')),
        (os.path.join('share', package_name, 'launch', 'amr_2'),
            glob('launch/amr_2/*.xml') + glob('launch/amr_2/*.py')),
        (os.path.join('share', package_name, 'launch', 'integration'),
            glob('launch/integration/*.xml') + glob('launch/integration/*.py')),
        (os.path.join('share', package_name, 'models', 'markers', 'apriltag_36h11_id0'),
            glob('models/markers/apriltag_36h11_id0/model.*')),
        (os.path.join('share', package_name, 'models', 'markers', 'apriltag_36h11_id0', 'meshes'),
            glob('models/markers/apriltag_36h11_id0/meshes/*')),
        (os.path.join('share', package_name, 'models', 'markers', 'checkerboard_8x6'),
            glob('models/markers/checkerboard_8x6/model.*')),
        (os.path.join('share', package_name, 'models', 'markers', 'checkerboard_8x6', 'meshes'),
            glob('models/markers/checkerboard_8x6/meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaejeongpark',
    maintainer_email='jjeongpark@gmail.com',
    description='Gazebo simulation launch package for just_pick_it project',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
