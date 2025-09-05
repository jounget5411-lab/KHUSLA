#!/usr/bin/env python3
import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- 1) 패키지 경로 ---
    pkg_mobile_robot = get_package_share_directory('mobile_robot')
    pkg_gps_path = get_package_share_directory('gps_path')

    # --- 2) 월드명 인자 (SDF <world name="...">와 일치) ---
    world_name = LaunchConfiguration('world_name')
    declare_world = DeclareLaunchArgument('world_name', default_value='driving_track_world')

    # --- 3) Xacro -> URDF ---
    xacro_path = os.path.join(pkg_mobile_robot, 'model', 'robot.xacro')
    robot_description = xacro.process_file(xacro_path).toxml()

    # --- 4) 월드 파일 경로 ---
    world_path = os.path.join(pkg_mobile_robot, 'worlds', 'my_world.sdf')

    # --- 5) (안정화) 리소스 경로 노출: mesh 등 package:// 경로 해석용 ---
    set_ign_res = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        f'{pkg_mobile_robot}:' + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    )
    set_gz_res = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        f'{pkg_mobile_robot}:' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    # --- 6) Gazebo 실행 ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # --- 7) 로봇 스폰 (월드 지정 + 이름 고정) ---
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_description,
            '-name', 'henes_t870',
            '-world', world_name,
            '-allow_renaming', 'false',
        ],
        output='screen',
    )
    # 초기화 여유
    spawn_after = TimerAction(period=6.0, actions=[spawn])

    # --- 8) Robot State Publisher ---
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    # --- 9) ROS <-> Gazebo 브리지 ---
    bridge_yaml_path = os.path.join(pkg_mobile_robot, 'parameters', 'bridge_parameters.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_yaml_path}'],
        output='screen',
    )

    # --- 10) Pose_V → 모델 포즈 어댑터(항상 살아있는 Pose_V에서 henes_t870만 추출) ---
    posev_adapter = Node(
        package='mobile_robot',
        executable='posev_to_model_pose.py',
        output='screen',
        parameters=[
            {'model_name': 'henes_t870'},
            {'out_topic': '/henes_t870/world_pose'},
            {'use_sim_time': True},
        ],
    )

    # --- 11) Gazebo 월드 포즈 -> NavSat 변환 노드 ---
    wp2gps = Node(
        package='mobile_robot',
        executable='worldpose_to_navsat.py',
        output='screen',
        parameters=[
            {'lat0_deg': 37.2889339},
            {'lon0_deg': 127.1076245},
            {'alt0_m': 114.193},
            {'heading_deg': 0.0},
            {'frame_id': 'gps_link'},
            {'use_sim_time': True},
        ],
    )

    # --- 12) 차량 제어 인터페이스 ---
    veh_if = Node(
        package='mobile_robot',
        executable='vehicle_interface.py',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # --- 13) GPS 중앙선 발행 노드 ---
    left_csv_path = os.path.join(pkg_gps_path, 'data', 'left_lane.csv')
    right_csv_path = os.path.join(pkg_gps_path, 'data', 'right_lane.csv')
    gps_centerline_node = Node(
        package='gps_path',
        executable='gps_centerline_node',
        name='gps_centerline_server',
        output='screen',
        parameters=[
            {'left_csv': left_csv_path},
            {'right_csv': right_csv_path},
            {'topic_centerline': '/gps/centerline'},
            {'use_sim_time': True},
        ],
    )

    # --- 14) 판단부 노드들 ---
    path_planner_node = Node(
        package='decision_making_pkg',
        executable='path_planner',
        output='screen',
        remappings=[('/ublox_gps_node/fix', '/gps/fix')],
        parameters=[{'use_sim_time': True}],
    )
    motion_planner_node = Node(
        package='decision_making_pkg',
        executable='motion_planner',
        output='screen',
        remappings=[('/ublox_gps_node/fix', '/gps/fix')],
        parameters=[{'use_sim_time': True}],
    )

    # --- 15) LaunchDescription ---
    return LaunchDescription([
        declare_world,
        set_ign_res, set_gz_res,  # 리소스 경로 노출
        gz_sim,
        spawn_after,
        rsp,
        bridge,
        posev_adapter,  # ★ 어댑터 추가
        wp2gps,
        gps_centerline_node,
        path_planner_node,
        motion_planner_node,
        veh_if,
    ])
