#!/usr/bin/env python3
import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- 1) 패키지 경로 ---
    pkg_mobile_robot = get_package_share_directory('mobile_robot')
    pkg_gps_path = get_package_share_directory('gps_path')
    pkg_decision_making = get_package_share_directory('decision_making_pkg')

    # --- 2) 월드명 인자 (SDF <world name="...">와 일치) ---
    world_name = LaunchConfiguration('world_name')
    declare_world = DeclareLaunchArgument('world_name', default_value='driving_track_world')

    # --- 3) Xacro -> URDF ---
    xacro_path = os.path.join(pkg_mobile_robot, 'model', 'robot.xacro')
    robot_description = xacro.process_file(xacro_path).toxml()

    # --- 4) 월드 파일 경로 ---
    world_path = os.path.join(pkg_mobile_robot, 'worlds', 'my_world.sdf')

    # --- 5) Gazebo 실행 ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # --- 6) 로봇 스폰 (월드 지정 + 이름 고정) ---
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
    spawn_after = TimerAction(period=5.0, actions=[spawn])

    # --- 7) Robot State Publisher ---
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    # --- 8) ROS <-> Gazebo 브리지 ---
    bridge_yaml_path = os.path.join(pkg_mobile_robot, 'parameters', 'bridge_parameters.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_yaml_path}'],
        output='screen',
    )

    # --- 9) Pose_V → 모델 포즈 어댑터 (ExecuteProcess로 직접 실행) ---
    posev_adapter = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(pkg_mobile_robot, 'scripts', 'posev_to_model_pose.py'),
            '--ros-args',
            '-p', 'model_name:=henes_t870',
            '-p', 'out_topic:=/henes_t870/world_pose',
            '-p', 'use_sim_time:=True',
        ],
        output='screen'
    )

    # --- 10) Gazebo 월드 포즈 -> NavSat 변환 노드 (ExecuteProcess로 직접 실행) ---
    wp2gps = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(pkg_mobile_robot, 'scripts', 'worldpose_to_navsat.py'),
            '--ros-args',
            '-p', 'lat0_deg:=37.2889339',
            '-p', 'lon0_deg:=127.1076245',
            '-p', 'alt0_m:=114.193',
            '-p', 'heading_deg:=0.0',
            '-p', 'frame_id:=gps_link',
            '-p', 'use_sim_time:=True',
        ],
        output='screen'
    )

    # --- 11) 차량 제어 인터페이스 ---
    veh_if = Node(
        package='mobile_robot',
        executable='vehicle_interface.py',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # --- 12) GPS 중앙선 발행 노드 ---
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

    # --- 13) 판단부 노드들 ---
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

    # --- 14) LaunchDescription ---
    return LaunchDescription([
        declare_world,
        gz_sim,
        spawn_after,
        rsp,
        bridge,
        posev_adapter,
        wp2gps,
        gps_centerline_node,
        path_planner_node,
        motion_planner_node,
        veh_if,
    ])
