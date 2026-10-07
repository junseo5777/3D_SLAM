from glob import glob
from setuptools import find_packages, setup

PACKAGE = 'macaron_parking'
setup(
    name=PACKAGE,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml', 'README.md', 'NOTICE']),
        ('share/' + PACKAGE + '/config', glob('config/*.yaml')),
        ('share/' + PACKAGE + '/launch', glob('launch/*.launch.py')),
        ('share/' + PACKAGE + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='junseo5777',
    maintainer_email='junseo5777@gmail.com',
    description='macaron8_HL용 PCD 주차 경로 계획과 추종',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'parking_node = macaron_parking.parking_node:main',
        'macaron_adapter = macaron_parking.adapter_node:main',
        'parking_plan = macaron_parking.offline:main',
    ]},
)
