import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import String, Bool, Float32
from interfaces_pkg.msg import PathPlanningResult, DetectionArray
from nav_msgs.msg import Odometry
from pyproj import CRS, Transformer
import math

#---------------Variable Setting---------------
SUB_DETECTION_TOPIC_NAME = "detections"
SUB_PATH_TOPIC_NAME = "/path_planning_result"
SUB_TRAFFIC_LIGHT_TOPIC_NAME = "yolov5_traffic_light_info"
SUB_LIDAR_OBSTACLE_TOPIC_NAME = "lidar_obstacle_info"
PUB_TOPIC_NAME = "topic_control_signal"

# ---- STOPLINE CONFIG (하드코딩) ----
# LLA 좌표 (위도, 경도), 나중에 map 좌표로 변환
STOPLINES = [
    (37.288796, 127.107228),
    (37.2887286, 127.1071174),
    (37.2886392, 127.1071944),
]
STOP_RADIUS_M_DEFAULT = 2.0  # 정지선 반경 기본값(튜닝 가능; 요구사항: 2.0 m)

# ---- SLOPE STOP CONFIG ----
# 경사로(램프) 정지 포인트들을 위경도(LLA)로 하드코딩하세요. 노드 시작 시 map (x,y)로 변환합니다.
SLOPE_STOPS = [
    (37.289035, 127.1073),
]
SLOPE_STOP_RADIUS_M_DEFAULT = 1.0   # 경사로 정지 반경 (m)
SLOPE_HOLD_SEC_DEFAULT      = 3.0   # 경사로 정지 유지 시간 (s)
# -----------------------------------

#----------------------------------------------

# 모션 플랜 발행 주기 (초) - 소수점 필요 (int형은 반영되지 않음)
TIMER = 0.1

class MotionPlanningNode(Node):
    def __init__(self):
        super().__init__('motion_planner_node')

        # 토픽 이름 설정
        self.sub_detection_topic = self.declare_parameter('sub_detection_topic', SUB_DETECTION_TOPIC_NAME).value
        self.sub_path_topic = self.declare_parameter('sub_path_topic', SUB_PATH_TOPIC_NAME).value
        self.sub_traffic_light_topic = self.declare_parameter('sub_traffic_light_topic', SUB_TRAFFIC_LIGHT_TOPIC_NAME).value
        self.sub_lidar_obstacle_topic = self.declare_parameter('sub_lidar_obstacle_topic', SUB_LIDAR_OBSTACLE_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value

        self.sub_odom_topic = self.declare_parameter('sub_odom_topic', '/odometry/filtered_map').value
        self.origin_latlon  = self.declare_parameter('origin_latlon', [37.28894785, 127.10763105]).value  # centerline/map 원점과 동일해야 함
        self.slope_stop_radius_m = float(self.declare_parameter('slope_stop_radius_m', SLOPE_STOP_RADIUS_M_DEFAULT).value)
        self.slope_hold_sec      = float(self.declare_parameter('slope_hold_sec',      SLOPE_HOLD_SEC_DEFAULT).value)

        self.timer_period = self.declare_parameter('timer', TIMER).value

        # ↓↓↓ 추가: vehicle_interface가 구독하는 토픽/타입 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        self.pub_steer = self.create_publisher(Float32, '/vehicle/steering_cmd', 10)
        self.pub_speed = self.create_publisher(Float32, '/vehicle/speed_cmd', 10)
        # vehicle_interface 기본 리미트와 동일하게 사용
        self.steer_lim_rad = float(self.declare_parameter('steer_limit_rad', 0.628).value)
        self.v_lim_mps     = float(self.declare_parameter('speed_limit_mps', 1.94).value)
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

        # QoS 설정
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # 변수 초기화
        self.detection_data = None
        self.path_data = None
        self.traffic_light_data = None
        self.lidar_data = None

        self.px = None
        self.py = None
        self.yaw = 0.0
        self.v = 0.0
        self.omega = 0.0

        self.steering_command = 0
        self.front_speed_command = 0
        self.rear_speed_command = 0
        
        self.stop_radius_m = float(self.declare_parameter('stop_radius_m', STOP_RADIUS_M_DEFAULT).value)
        self.stoplines = list(STOPLINES)

        # 교차로 FSM 상태
        self.stopped = False
        self.current_stop_idx = None  # 0,1,2 중 하나 또는 None
        self.completed_stops = set()  # 이미 처리 완료한 정지선 인덱스

        self.slope_stop_active = False
        self.slope_stop_until  = 0.0

        # 좌표 변환기 준비 (EPSG:4326 -> 로컬 AEQD(map))
        try:
            lat0, lon0 = float(self.origin_latlon[0]), float(self.origin_latlon[1])
            crs_geod = CRS.from_epsg(4326)
            crs_map  = CRS.from_proj4(f"+proj=aeqd +lat_0={lat0} +lon_0={lon0} +datum=WGS84 +units=m +no_defs")
            self._lla2map = Transformer.from_crs(crs_geod, crs_map, always_xy=True)
        except Exception as e:
            raise RuntimeError(f"좌표 변환기 초기화 실패(origin_latlon={self.origin_latlon}): {e}")
        
        def _lla_to_map(lat, lon):
            x, y = self._lla2map.transform(lon, lat)  # (lon,lat) 순서 주의
            return float(x), float(y)
        
        # STOPLINES (LLA) -> map (x,y)
        self.stoplines_xy = [_lla_to_map(lat, lon) for (lat, lon) in self.stoplines]
        # SLOPE STOPS (LLA) -> map (x,y)
        self.slope_stops_xy = [_lla_to_map(lat, lon) for (lat, lon) in SLOPE_STOPS]

        # 서브스크라이버 설정
        self.detection_sub = self.create_subscription(DetectionArray, self.sub_detection_topic, self.detection_callback, self.qos_profile)
        self.path_sub = self.create_subscription(PathPlanningResult, self.sub_path_topic, self.path_callback, self.qos_profile)
        self.traffic_light_sub = self.create_subscription(String, self.sub_traffic_light_topic, self.traffic_light_callback, self.qos_profile)
        self.lidar_sub = self.create_subscription(Bool, self.sub_lidar_obstacle_topic, self.lidar_callback, self.qos_profile)
        self.odom_sub = self.create_subscription(Odometry, self.sub_odom_topic, self.odom_callback, self.qos_profile)

        # 퍼블리셔 설정
        self.publisher = self.create_publisher(String, self.pub_topic, self.qos_profile)

        # 타이머 설정
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.px = float(p.x)
        self.py = float(p.y)
        # quaternion -> yaw (Z) without tf_transformations
        w, x, y, z = float(q.w), float(q.x), float(q.y), float(q.z)
        self.yaw = math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))
        # twist가 제공되면 사용 (없으면 0 유지)
        try:
            self.v = float(msg.twist.twist.linear.x)
            self.omega = float(msg.twist.twist.angular.z)
        except Exception:
            pass

    def detection_callback(self, msg: DetectionArray):
        self.detection_data = msg

    def path_callback(self, msg: PathPlanningResult):
        self.path_data = list(zip(msg.x_points, msg.y_points))
                
    def traffic_light_callback(self, msg: String):
        self.traffic_light_data = msg

    def lidar_callback(self, msg: Bool):
        self.lidar_data = msg

    # ↓↓↓ 추가 헬퍼: 정수 명령 → Float32 토픽으로 변환/발행
    def _publish_vehicle_cmds(self):
        steer_rad = float(self.steering_command) / 3.0 * self.steer_lim_rad
        speed_mps = float(self.front_speed_command) / 100.0 * self.v_lim_mps
        self.pub_steer.publish(Float32(data=steer_rad))
        self.pub_speed.publish(Float32(data=speed_mps))
    # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    def _is_near_any_stopline(self):
        if self.px is None or self.py is None:
            return False, None, None
        if not self.stoplines_xy:
            return False, None, None
        min_d2 = None
        min_stop = None
        for (sx, sy) in self.stoplines_xy:
            dx = sx - self.px
            dy = sy - self.py
            d2 = dx*dx + dy*dy
            if (min_d2 is None) or (d2 < min_d2):
                min_d2 = d2
                min_stop = (sx, sy)
        if min_d2 is None:
            return False, None, None
        dist = math.sqrt(min_d2)
        return (dist <= self.stop_radius_m), dist, min_stop

    def _nearest_stopline_within_radius(self, px, py, radius_m):
        if px is None or py is None:
            return None, None
        if not self.stoplines_xy:
            return None, None
        best_idx = None
        best_d2 = None
        for idx, (sx, sy) in enumerate(self.stoplines_xy):
            if idx in self.completed_stops:
                continue
            dx = sx - px
            dy = sy - py
            d2 = dx*dx + dy*dy
            if d2 <= radius_m * radius_m:
                if (best_d2 is None) or (d2 < best_d2):
                    best_d2 = d2
                    best_idx = idx
        if best_idx is None:
            return None, None
        return best_idx, math.sqrt(best_d2)

    def _traffic_is(self, want: str) -> bool:
        if self.traffic_light_data is None:
            return False
        val = str(self.traffic_light_data.data).strip()
        return val.lower() == want.lower()

    def _should_trigger_slope_stop(self):
        # 이미 활성화 중이면 그대로 유지
        if self.slope_stop_active:
            return True
        if self.px is None or self.py is None:
            return False
        for (sx, sy) in self.slope_stops_xy:
            dx = sx - self.px
            dy = sy - self.py
            d = math.hypot(dx, dy)
            if d <= self.slope_stop_radius_m:
                return True
        return False

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9  # seconds
        # 경사로 정지 우선 적용
        if self.slope_stop_active:
            if now < self.slope_stop_until:
                self.steering_command = 0
                self.front_speed_command = 0
                self.rear_speed_command = 0
                self.get_logger().info(f"SLOPE-STOP active: holding for {self.slope_stop_until - now:.1f}s")
                serial_cmd = String()
                serial_cmd.data = f"s{self.steering_command}m{self.front_speed_command}"
                self._publish_vehicle_cmds()            # ← 추가
                self.publisher.publish(serial_cmd)
                return
            else:
                # 홀드 종료
                self.slope_stop_active = False
        
        # 새로 진입했는지 확인
        if self._should_trigger_slope_stop():
            self.slope_stop_active = True
            self.slope_stop_until = now + self.slope_hold_sec
            self.steering_command = 0
            self.front_speed_command = 0
            self.rear_speed_command = 0
            self.get_logger().info(f"SLOPE-STOP triggered: hold {self.slope_hold_sec:.1f}s (radius={self.slope_stop_radius_m} m)")
            serial_cmd = String()
            serial_cmd.data = f"s{self.steering_command}m{self.front_speed_command}"
            self._publish_vehicle_cmds()                # ← 추가
            self.publisher.publish(serial_cmd)
            return

        # ---- 교차로 FSM: 반경 2m AND Red에서만 정지, 지점별 재출발 ----
        near_idx, near_dist = self._nearest_stopline_within_radius(self.px, self.py, self.stop_radius_m)
        
        if not self.stopped:
            # 정지 트리거: 반경 내 AND Red
            if (near_idx is not None) and self._traffic_is('Red'):
                self.stopped = True
                self.current_stop_idx = near_idx
                self.steering_command = 0
                self.front_speed_command = 0
                self.rear_speed_command = 0
                self.get_logger().info(f"STOP (idx={near_idx}, dist={near_dist:.2f} m, reason=Red&radius≤{self.stop_radius_m}m)")
                serial_cmd = String()
                serial_cmd.data = f"s{self.steering_command}m{self.front_speed_command}"
                self._publish_vehicle_cmds()            # ← 추가
                self.publisher.publish(serial_cmd)
                return
        else:
            # 정지 중: 지점별 신호에 따라 재출발
            idx = self.current_stop_idx
            if idx is not None:
                need = 'Green' if idx in (0, 1) else 'Left'
                if self._traffic_is(need):
                    self.stopped = False
                    self.completed_stops.add(idx)
                    self.current_stop_idx = None
                    self.get_logger().info(f"RESUME (idx={idx}, signal={need})")
                else:
                    # 신호 대기 계속 정지
                    self.steering_command = 0
                    self.front_speed_command = 0
                    self.rear_speed_command = 0
                    serial_cmd = String()
                    serial_cmd.data = f"s{self.steering_command}m{self.front_speed_command}"
                    self._publish_vehicle_cmds()        # ← 추가
                    self.publisher.publish(serial_cmd)
                    return

        if self.lidar_data is not None and self.lidar_data.data is True:
            # 라이다가 장애물을 감지한 경우
            self.steering_command = 0 
            self.front_speed_command = 0 
            self.rear_speed_command = 0 

        else:
            # GPS 경로 추종 (x=우측+, y=전방+ 프레임 가정)
            if self.path_data is None or len(self.path_data) < 2:
                # 경로가 없거나 너무 짧으면 정지/보호
                self.steering_command = 0
                self.front_speed_command = 0
                self.rear_speed_command = 0
            else:
                # 마지막 구간의 방향(헤딩 오차)으로 7단계 조향(1~7) 결정
                start_idx = max(0, len(self.path_data) - 10)
                p1 = self.path_data[start_idx]
                p2 = self.path_data[-1]

                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]

                # 차량 전방이 +y이므로 좌/우 편차는 atan2(dx, dy)
                if dx == 0.0 and dy == 0.0:
                    heading_err = 0.0
                else:
                    heading_err = math.atan2(dx, dy)

                # 최대 허용 각도를 기준으로 7단계(−3..+3) 양자화
                # 예: ±40도를 풀스케일로 가정
                MAX_ANGLE = math.radians(40.0)
                raw_level = round((heading_err / MAX_ANGLE) * 3)
                raw_level = max(-3, min(3, raw_level))  # -3..+3로 클램프

                # 최종 조향 명령: -3(최좌) ~ 0(직진) ~ +3(최우)
                self.steering_command = int(raw_level)

                # 정상 경로일 때만 속도 명령
                self.front_speed_command = 100
                self.rear_speed_command = 100

        self.get_logger().info(f"steering: {self.steering_command}, " 
                               f"front_speed: {self.front_speed_command}, " 
                               f"rear_speed: {self.rear_speed_command}")

        # 압축 문자열 형식으로 퍼블리시: s{steering}m{front_speed}
        # 예: steering=-3, front=100 -> "s-3m100"
        serial_cmd = String()
        serial_cmd.data = f"s{self.steering_command}m{self.front_speed_command}"
        self._publish_vehicle_cmds()                    # ← 추가
        self.publisher.publish(serial_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
