import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy


from std_msgs.msg import String, Bool, Float32
from interfaces_pkg.msg import PathPlanningResult, DetectionArray
from sensor_msgs.msg import NavSatFix
from .lib import decision_making_func_lib as DMFL
import math

#---------------Variable Setting---------------
SUB_DETECTION_TOPIC_NAME = "detections"
SUB_PATH_TOPIC_NAME = "/path_planning_result"
SUB_TRAFFIC_LIGHT_TOPIC_NAME = "yolov5_traffic_light_info"
SUB_LIDAR_OBSTACLE_TOPIC_NAME = "lidar_obstacle_info"
PUB_TOPIC_NAME = "topic_control_signal"

# ---- STOPLINE CONFIG (하드코딩) ----
# TODO: 아래 좌표 4개를 교차로 정지선의 실제 GPS로 바꾸세요.
STOPLINES = [
    (37.288950, 127.107640),
    (37.289120, 127.107880),
    (37.289300, 127.108050),
    (37.289500, 127.108200),
]
STOP_RADIUS_M_DEFAULT = 6.0  # 정지선 반경 기본값(튜닝 가능)
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
    #    self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        self.sub_gps_fix_topic = self.declare_parameter('sub_gps_fix_topic', '/ublox_gps_node/fix').value
        
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

        self.current_lat = None
        self.current_lon = None

        self.steering_command = 0
        self.front_speed_command = 0
        self.rear_speed_command = 0
        
        self.stop_radius_m = float(self.declare_parameter('stop_radius_m', STOP_RADIUS_M_DEFAULT).value)
        self.stoplines = list(STOPLINES)

        # 서브스크라이버 설정
        self.detection_sub = self.create_subscription(DetectionArray, self.sub_detection_topic, self.detection_callback, self.qos_profile)
        self.path_sub = self.create_subscription(PathPlanningResult, self.sub_path_topic, self.path_callback, self.qos_profile)
        self.traffic_light_sub = self.create_subscription(String, self.sub_traffic_light_topic, self.traffic_light_callback, self.qos_profile)
        self.lidar_sub = self.create_subscription(Bool, self.sub_lidar_obstacle_topic, self.lidar_callback, self.qos_profile)
        self.gps_sub = self.create_subscription(NavSatFix, self.sub_gps_fix_topic, self.gps_callback, self.qos_profile)

        # 퍼블리셔 설정
        # self.publisher = self.create_publisher(String, self.pub_topic, self.qos_profile)
        self.speed_pub = self.create_publisher(Float32, '/vehicle/speed_cmd', self.qos_profile)
        self.steer_pub = self.create_publisher(Float32, '/vehicle/steering_cmd', self.qos_profile)

        # 타이머 설정
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def _haversine_m(self, lat1, lon1, lat2, lon2):
        R = 6371000.0
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat/2.0)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2.0)**2
        return 2.0 * R * math.asin(math.sqrt(a))

    def _is_near_any_stopline(self):
        if self.current_lat is None or self.current_lon is None:
            return False, None, None
        if not self.stoplines:
            return False, None, None
        min_dist = None
        min_stop = None
        for (slat, slon) in self.stoplines:
            d = self._haversine_m(self.current_lat, self.current_lon, slat, slon)
            if (min_dist is None) or (d < min_dist):
                min_dist = d
                min_stop = (slat, slon)
        return (min_dist is not None and min_dist <= self.stop_radius_m), min_dist, min_stop

    def gps_callback(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def detection_callback(self, msg: DetectionArray):
        self.detection_data = msg

    def path_callback(self, msg: PathPlanningResult):
        self.path_data = list(zip(msg.x_points, msg.y_points))
                
    def traffic_light_callback(self, msg: String):
        self.traffic_light_data = msg

    def lidar_callback(self, msg: Bool):
        self.lidar_data = msg
        
    def timer_callback(self):

        if self.lidar_data is not None and self.lidar_data.data is True:
            # 라이다가 장애물을 감지한 경우
            self.steering_command = 0 
            self.front_speed_command = 0 
            self.rear_speed_command = 0 

        elif self.traffic_light_data is not None and self.traffic_light_data.data == 'Red':
            # 빨간불 AND 정지선 반경 내일 때만 정지
            is_near, dist, which = self._is_near_any_stopline()
            if is_near:
                self.steering_command = 0
                self.front_speed_command = 0
                self.rear_speed_command = 0
                if dist is not None and which is not None:
                    self.get_logger().info(f"STOP (Red & near stopline): dist={dist:.2f} m near {which}")
            else:
                # 정지선에서 멀면 멈추지 않고 경로 추종 계속
                pass
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



#       self.get_logger().info(f"steering: {self.steering_command}, " 
#                              f"front_speed: {self.front_speed_command}, " 
#                              f"rear_speed: {self.rear_speed_command}")
#
#       # 압축 문자열 형식으로 퍼블리시: s{steering}m{front_speed}
#       # 예: steering=-3, front=100 -> "s-3m100"
#       serial_cmd = String()
#       serial_cmd.data = f"s{self.steering_command}m{self.front_speed_command}"
#       self.publisher.publish(serial_cmd)
        # 1. 실제 차량 스펙에 맞춰 값 변환(매핑)
        # 조향: -3~3 정수 단계를 -0.628 ~ +0.628 라디안으로 변환
        MAX_STEER_RAD = 0.628  # 36도
        steering_rad = float(self.steering_command) / 3.0 * MAX_STEER_RAD

        # 속도: 0~100 값을 0 ~ 1.94 m/s (7km/h) 로 변환
        MAX_SPEED_MPS = 1.94  # 7km/h
        speed_mps = float(self.front_speed_command) / 100.0 * MAX_SPEED_MPS

        self.get_logger().info(f"steering_cmd: {steering_rad:.3f} rad, "
                               f"speed_cmd: {speed_mps:.2f} m/s")

        # 2. 두 개의 Float32 메시지로 각각 발행
        speed_msg = Float32()
        speed_msg.data = speed_mps
        self.speed_pub.publish(speed_msg)

        steer_msg = Float32()
        steer_msg.data = steering_rad
        self.steer_pub.publish(steer_msg)        


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
