import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # RViz2를 GUI 모드로 실행할지 결정하는 파라미터
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 1. URDF 파일의 전체 경로를 찾습니다.
    urdf_file_name = 'my_car.urdf'
    urdf = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # 2. robot_state_publisher 노드를 설정합니다.
    # 이 노드는 URDF 파일을 읽어서 로봇의 각 관절과 링크(link)의 관계를
    # ROS 2 시스템 전체에 broadcast(방송)하는 역할을 합니다.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}]
    )

    # 3. RViz2 노드를 설정합니다.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    # 실행할 노드 리스트를 반환합니다.
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        robot_state_publisher_node,
        rviz_node
    ])
