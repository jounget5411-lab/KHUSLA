import os
from glob import glob # <--- glob 임포트 추가
from setuptools import find_packages, setup
package_name = 'gps_path'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 아래 data 폴더 설치 구문을 추가합니다.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'data'), glob('data/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DaeHyeon Kim',
    maintainer_email='dh08080@khu.ac.kr',
    description='Publish centerline Path from left/right lane CSVs',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_centerline_node = gps_path.gps_centerline_node:main',
        ],
    },
)