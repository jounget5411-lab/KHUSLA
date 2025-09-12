from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_path',
            executable='gps_centerline_node',
            name='gps_centerline_server',
            output='screen',
            parameters=[
                # 하드코딩 경로를 쓰지 않는다면 아래 파라미터를 켜서 사용
                # {'left_csv': '~/ws_mobile/src/gps_path/data/left_lane.csv'},
                # {'right_csv': '/home/daehyeon/ros2_ws/right_lane.csv'},
                {'frame_id': 'map'},
                {'spacing_m': 0.30},
                {'publish_rate_hz': 2.0},
                {'topic_centerline': '/gps/centerline'},
            ],
        ),
    ])