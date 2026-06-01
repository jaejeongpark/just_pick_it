from setuptools import find_packages, setup

package_name = 'jetcobot_sorting'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'async_test = jetcobot_sorting.async_test:main',
            'visual_servoing = jetcobot_sorting.visual_servoing:main',
            'fake_yolo_detection_publisher = jetcobot_sorting.fake_yolo_detection_publisher:main',
        ],
    },
)
