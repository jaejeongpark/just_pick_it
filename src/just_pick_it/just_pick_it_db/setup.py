from setuptools import find_packages, setup

package_name = 'just_pick_it_db'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'sqlalchemy>=2.0', 'psycopg2-binary'],
    zip_safe=True,
    maintainer='suwoo3131',
    maintainer_email='suwoo3131@gmail.com',
    description='just_pick_it 공용 DB 계층: ORM 모델, 세션 관리, 비즈니스 로직',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
)
