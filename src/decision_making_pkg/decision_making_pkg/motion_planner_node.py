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
SLOPE_STOPS = [
    (37.289035, 127.1073),
]
SLOPE_STOP_RADIUS_M_DEFAULT = 1.0
SLOPE_HOLD_SEC_DEFAULT      = 3.0
# -----------------------------------

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

        self.sub_odom_topic = self.declare_parameter('sub_odom_topic', '/odometry/filtered').value
        self.origin_latlon  = self.declare_parameter('origin_latlon', [37.28894785, 127.10763105]).value
        self.slope_stop_radius_m = float(self.declare_parameter('slope_stop_radius_m', SLOPE_STOP_RADIUS_M_DEFAULT).value)
        self.slope_hold_sec      = float(self.declare_parameter('slope_hold_sec',      SLOPE_HOLD_SEC_DEFAULT).value)

        self.timer_period = self.declare_parameter('timer', TIMER).value

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
        self.current_stop_idx = None
        self.completed_stops = set()

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

        # ADDED: vehicle_interface 제어 토픽 퍼블리셔
        self.pub_speed = self.create_publisher(Float32, '/vehicle/speed_cmd', self.qos_profile)   # ADDED
        self.pub_steer = self.create_publisher(Float32, '/vehicle/steering_cmd', self.qos_profile) # ADDED

        # 타이머 설정
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.px = float(p.x)
        self.py = float(p.y)
        # quaternion -> yaw (Z)
        w, x, y, z = float(q.w), float(q.x), float(q.y), float(q.z)
        self.yaw = math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))
        # twist
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

    def _publish_vehicle_cmd(self, steer_level: int, front_pct: int):  # ADDED helper
        """-3..+3 조향 단계와 0..100 속도%를 vehicle_interface 스케일로 변환해 퍼블리시"""
        STEER_LIMIT = 0.628   # rad (vehicle_interface와 동일)
        SPEED_LIMIT = 1.94    # m/s
        delta_rad = float(steer_level) / 3.0 * STEER_LIMIT
        speed_mps = float(front_pct) / 100.0 * SPEED_LIMIT
        self.pub_steer.publish(Float32(data=delta_rad))
        self.pub_speed.publish(Float32(data=speed_mps))

    def _publish_stop(self):  # ADDED helper
        """즉시 정지 명령 퍼블리시 (vehicle_interface + 기존 문자열 토픽 상호 일관)"""
        self.pub_steer.publish(Float32(data=0.0))
        self.pub_speed.publish(Float32(data=0.0))

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
                self.publisher.publish(serial_cmd)
                self._publish_stop()  # ADDED
                return
            else:
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
            self.publisher.publish(serial_cmd)
            self._publish_stop()  # ADDED
            return

        # ---- 교차로 FSM ----
        near_idx, near_dist = self._nearest_stopline_within_radius(self.px, self.py, self.stop_radius_m)
        
        if not self.stopped:
            if (near_idx is not None) and self._traffic_is('Red'):
                self.stopped = True
                self.current_stop_idx = near_idx
                self.steering_command = 0
                self.front_speed_command = 0
                self.rear_speed_command = 0
                self.get_logger().info(f"STOP (idx={near_idx}, dist={near_dist:.2f} m, reason=Red&radius≤{self.stop_radius_m}m)")
                serial_cmd = String()
                serial_cmd.data = f"s{self.steering_command}m{self.front_speed_command}"
                self.publisher.publish(serial_cmd)
                self._publish_stop()  # ADDED
                return
        else:
            idx = self.current_stop_idx
            if idx is not None:
                need = 'Green' if idx in (0, 1) else 'Left'
                if self._traffic_is(need):
                    self.stopped = False
                    self.completed_stops.add(idx)
                    self.current_stop_idx = None
                    self.get_logger().info(f"RESUME (idx={idx}, signal={need})")
                else:
                    self.steering_command = 0
                    self.front_speed_command = 0
                    self.rear_speed_command = 0
                    serial_cmd = String()
                    serial_cmd.data = f"s{self.steering_command}m{self.front_speed_command}"
                    self.publisher.publish(serial_cmd)
                    self._publish_stop()  # ADDED
                    return

        if self.lidar_data is not None and self.lidar_data.data is True:
            self.steering_command = 0 
            self.front_speed_command = 0 
            self.rear_speed_command = 0 
        else:
            # GPS 경로 추종
            if self.path_data is None or len(self.path_data) < 2:
                self.steering_command = 0
                self.front_speed_command = 0
                self.rear_speed_command = 0
            else:
                start_idx = max(0, len(self.path_data) - 10)
                p1 = self.path_data[start_idx]
                p2 = self.path_data[-1]

                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]

                if dx == 0.0 and dy == 0.0:
                    heading_err = 0.0
                else:
                    heading_err = math.atan2(dx, dy)

                MAX_ANGLE = math.radians(40.0)
                raw_level = round((heading_err / MAX_ANGLE) * 3)
                raw_level = max(-3, min(3, raw_level))

                self.steering_command = int(raw_level)
                self.front_speed_command = 100
                self.rear_speed_command = 100

        self.get_logger().info(
            f"steering: {self.steering_command}, front_speed: {self.front_speed_command}, rear_speed: {self.rear_speed_command}"
        )

        # 기존 압축 문자열 퍼블리시(유지)
        serial_cmd = String()
        serial_cmd.data = f"s{self.steering_command}m{self.front_speed_command}"
        self.publisher.publish(serial_cmd)

        # ADDED: 실제 vehicle_interface 제어 퍼블리시
        self._publish_vehicle_cmd(self.steering_command, self.front_speed_command)

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
