from setuptools import find_packages, setup

package_name = 'pinky_amr_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/params', ['params/reverse_docking.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='suwoo3131',
    maintainer_email='suwoo3131@gmail.com',
    description='AMR State Manager for just_pick_it system',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'state_manager = pinky_amr_1.state_manager:main',
        ],
    },
)
