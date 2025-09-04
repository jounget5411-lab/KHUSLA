from setuptools import find_packages, setup

package_name = 'decision_making_pkg'

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
    maintainer='your_name', # 이 부분은 사용자님 정보로 변경하시는 것을 추천합니다.
    maintainer_email='your_email@email.com', # 이 부분은 사용자님 정보로 변경하시는 것을 추천합니다.
    description='Path and Motion planning nodes for the autonomous vehicle',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # '실행명령어 = 파이썬파일경로:main함수' 형식
            'path_planner = decision_making_pkg.path_planner_node:main',
            'motion_planner = decision_making_pkg.motion_planner_node:main',
        ],
    },
)
