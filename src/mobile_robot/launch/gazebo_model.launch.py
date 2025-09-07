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
    # --- 패키지 경로 ---
    pkg_mobile_robot = get_package_share_directory('mobile_robot')
    pkg_gps_path     = get_package_share_directory('gps_path')

    # --- 런치 인자: 월드명/스폰 포즈 ---
    world_name = LaunchConfiguration('world_name')
    declare_world = DeclareLaunchArgument('world_name', default_value='driving_track_world')

    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_Y = LaunchConfiguration('spawn_Y')  # yaw [rad]

    declare_spawn_x = DeclareLaunchArgument('spawn_x', default_value='-3.68')   # 계산치
    declare_spawn_y = DeclareLaunchArgument('spawn_y', default_value='2.897')   # 계산치
    declare_spawn_z = DeclareLaunchArgument('spawn_z', default_value='0.0')
    declare_spawn_Y = DeclareLaunchArgument('spawn_Y', default_value='2.8')     # 라디안

    # --- Xacro → URDF ---
    xacro_path = os.path.join(pkg_mobile_robot, 'model', 'robot.xacro')
    robot_description = xacro.process_file(xacro_path).toxml()

    # --- 월드 파일 ---
    world_path = os.path.join(pkg_mobile_robot, 'worlds', 'my_world.sdf')

    # --- 리소스 경로 노출(안정화) ---
    set_ign_res = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        f'{pkg_mobile_robot}:' + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    )
    set_gz_res = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        f'{pkg_mobile_robot}:' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    # --- Gazebo 실행 ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # --- 로봇 스폰 (월드/포즈 명시) ---
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_description,
            '-name',   'henes_t870',
            '-world',  world_name,
            '-x',      spawn_x,
            '-y',      spawn_y,
            '-z',      spawn_z,
            '-Y',      spawn_Y,      # yaw[rad]
            '-allow_renaming', 'false',
        ],
        output='screen',
    )
    spawn_after = TimerAction(period=6.0, actions=[spawn])

    # --- RSP ---
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    # --- Bridge ---
    bridge_yaml_path = os.path.join(pkg_mobile_robot, 'parameters', 'bridge_parameters.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_yaml_path}'],
        output='screen',
    )

    # --- Pose_V → 모델 Pose 어댑터 ---
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

    # --- world_pose → NavSat (GPS) ---
    wp2gps = Node(
        package='mobile_robot',
        executable='worldpose_to_navsat.py',
        output='screen',
        parameters=[
            {'lat0_deg': 37.2889339},
            {'lon0_deg': 127.1076245},
            {'alt0_m':   114.193},
            {'heading_deg': 0.0},
            {'frame_id': 'gps_link'},
            {'std_xy_m': 0.15},
            {'std_z_m':  0.30},
            {'use_sim_time': True},
        ],
    )

    # === IMU 공분산 주입 노드 ===
    imu_cov = Node(
        package='mobile_robot',
        executable='imu_cov_injector.py',
        output='screen',
        parameters=[
            {'in_topic': '/imu'},
            {'out_topic': '/imu/with_cov'},
            {'ori_var':  9.0e-4},  # (0.03 rad)^2
            {'gyro_var': 4.0e-4},  # (0.02 rad/s)^2
            {'acc_var':  2.5e-3},  # (0.05 m/s^2)^2
            {'use_sim_time': True},
        ],
    )

    # =========================
    # EKF + NavSat Transform
    # =========================
    ekf_yaml    = os.path.join(pkg_mobile_robot, 'parameters', 'ekf_local.yaml')
    navsat_yaml = os.path.join(pkg_mobile_robot, 'parameters', 'navsat.yaml')

    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[ekf_yaml, {'use_sim_time': True}],
        # YAML에서 imu0가 이미 /imu/with_cov라면 아래 remap은 생략 가능
        # remappings=[('/imu', '/imu/with_cov')],
    )

    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        remappings=[
            # IMU는 /imu 또는 /imu/data 어느쪽을 쓰는 빌드인지 케이스가 갈려서 둘 다 매핑
            ('/imu', '/imu/with_cov'),
            ('/imu/data', '/imu/with_cov'),

            ('/gps/fix', '/gps/fix'),
            ('/odometry/filtered', '/odometry/filtered'),

            # ★ 충돌 원인 제거: Odometry는 별도 이름으로
            ('/odometry/gps', '/gps/odom'),   # <- 새 이름
            # ('/gps/filtered', '/gps/filtered')  # NavSatFix는 디폴트 그대로 사용
        ],
        parameters=[navsat_yaml, {'use_sim_time': True}],
    )



    # --- 차량 인터페이스 ---
    veh_if = Node(
        package='mobile_robot',
        executable='vehicle_interface.py',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # --- 중앙선 발행 ---
    left_csv_path  = os.path.join(pkg_gps_path, 'data', 'left_lane.csv')
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

    # --- 판단부 ---
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

    return LaunchDescription([
        declare_world, declare_spawn_x, declare_spawn_y, declare_spawn_z, declare_spawn_Y,
        set_ign_res, set_gz_res,
        gz_sim,
        spawn_after,
        rsp,
        bridge,
        posev_adapter,
        wp2gps,
        imu_cov,
        ekf_local,
        navsat,               # ← 한 번만! (navsat_after 제거)
        gps_centerline_node,
        path_planner_node,
        motion_planner_node,
        veh_if,
    ])
