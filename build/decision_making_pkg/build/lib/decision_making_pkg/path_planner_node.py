import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
)

from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from interfaces_pkg.msg import PathPlanningResult

import pyproj

# --------------- Parameters (defaults) ---------------
SUB_PATH_TOPIC_NAME = "/gps/centerline"     # gps_centerline_node가 퍼블리시하는 Path (frame: map, 단위: m)
SUB_GPS_TOPIC_NAME  = "/ublox_gps_node/fix"  # u-blox 드라이버가 퍼블리시하는 표준 NavSatFix (deg/m)
PUB_TOPIC_NAME      = "/path_planning_result" # motion_planner가 구독하는 기존 토픽/메시지 유지

ORIGIN_LAT = 37.28894785  # hardcoded origin (avg of first left/right CSV points)
ORIGIN_LON = 127.10763105

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        # ---- 파라미터 선언 (최소 변경) ----
        self.sub_path_topic = self.declare_parameter('sub_path_topic', SUB_PATH_TOPIC_NAME).value
        self.sub_gps_topic  = self.declare_parameter('sub_gps_topic', SUB_GPS_TOPIC_NAME).value
        self.pub_topic      = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value

        # hardcoded projection origin (must match gps_centerline_node)
        self.origin_lat = ORIGIN_LAT
        self.origin_lon = ORIGIN_LON

        # 앞으로 사용할 경로 길이/간격 제어(선택)
        self.ahead_len = int(self.declare_parameter('ahead_len', 50).value)   # 앞쪽으로 보낼 점 개수
        self.stride    = int(self.declare_parameter('downsample_stride', 1).value)  # 다운샘플 간격(1=그대로)

        # ---- QoS ----
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )

        # ---- 퍼블리셔/서브스크라이버 ----
        self.publisher = self.create_publisher(PathPlanningResult, self.pub_topic, self.qos_profile)
        self.path_sub  = self.create_subscription(Path,      self.sub_path_topic, self._on_path, self.qos_profile)
        self.gps_sub   = self.create_subscription(NavSatFix, self.sub_gps_topic,  self._on_fix,  self.qos_profile)

        # ---- 내부 상태 ----
        self._last_path = None     # nav_msgs/Path
        self._have_fix  = False
        self._fix_lat   = None
        self._fix_lon   = None

        # 투영기 준비(가능하면 pyproj 사용)
        self._proj = self._make_proj(self.origin_lat, self.origin_lon)

        self.get_logger().info(
            f"path_planner_node ready (HARD-CODED ORIGIN): sub_path='{self.sub_path_topic}', sub_gps='{self.sub_gps_topic}', "
            f"pub='{self.pub_topic}', origin=({self.origin_lat}, {self.origin_lon}), ahead_len={self.ahead_len}, stride={self.stride}, proj=AEQD(pyproj-only)"
        )

    # ---------------- Callbacks ----------------
    def _on_path(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn("Received empty Path; skip")
            return
        self._last_path = msg
        self._try_publish()

    def _on_fix(self, msg: NavSatFix):
        if msg.status.status < 0:
            # 무효한 GPS 상태면 사용하지 않음
            self._have_fix = False
            return
        self._fix_lat = float(msg.latitude)
        self._fix_lon = float(msg.longitude)
        self._have_fix = True
        self._try_publish()

    # --------------- Core logic ---------------
    def _try_publish(self):
        if (self._last_path is None) or (not self._have_fix):
            return

        # 1) 현재 위치(LLA) -> map 프레임 (m)
        x0, y0 = self._latlon_to_xy(self._fix_lat, self._fix_lon)

        # 2) Path를 배열로 추출
        xs, ys = self._path_to_arrays(self._last_path)
        if xs.size < 2:
            self.get_logger().warn("Path has <2 points; skip")
            return

        # 3) 현재 위치에 가장 가까운 점 찾기
        idx = self._nearest_index(xs, ys, x0, y0)

        # 4) 앞 구간 선택 + 다운샘플
        start = idx
        end   = min(len(xs), start + max(2, self.ahead_len * max(1, self.stride)))
        xs_seg = xs[start:end:self.stride]
        ys_seg = ys[start:end:self.stride]
        if xs_seg.size < 2:
            self.get_logger().warn("Too few forward points after slicing; skip")
            return

        # 5) 로컬(차량 유사) 좌표로 변환
        #    - 원점: 현재 위치(x0, y0)
        #    - +y가 경로 진행방향이 되도록 회전(차량 헤딩이 없어도 경로 접선 사용)
        #    경로 접선 각도
        j = min(start + 5, len(xs) - 1)
        theta = math.atan2(ys[j] - ys[start], xs[j] - xs[start])  # w.r.t +x
        # 경로 진행방향이 +y가 되도록 회전각(alpha) = pi/2 - theta
        alpha = (math.pi / 2.0) - theta
        ca = math.cos(alpha)
        sa = math.sin(alpha)

        dx = xs_seg - x0
        dy = ys_seg - y0
        x_local = ca * dx - sa * dy
        y_local = sa * dx + ca * dy

        # y가 증가하는 방향으로 정렬 보장(혹시 역방향이면 뒤집기)
        if y_local[-1] < y_local[0]:
            x_local = x_local[::-1]
            y_local = y_local[::-1]

        # 6) motion_planner가 기대하는 형식으로 퍼블리시 (x=오른쪽, y=전방)
        out = PathPlanningResult()
        out.x_points = [float(v) for v in x_local]
        out.y_points = [float(v) for v in y_local]
        self.publisher.publish(out)

        self.get_logger().info(
            f"Published PathPlanningResult: {len(out.x_points)} pts (nearest_idx={idx}, alpha={alpha:.3f} rad)"
        )

    # --------------- Utilities ---------------
    def _make_proj(self, lat0: float, lon0: float):
        try:
            return pyproj.Proj(proj='aeqd', lat_0=lat0, lon_0=lon0, datum='WGS84', units='m')
        except Exception as e:
            # pyproj가 필수이므로 실패 시 즉시 중단
            raise RuntimeError(f"pyproj.Proj init failed: {e}")

    def _latlon_to_xy(self, lat_deg: float, lon_deg: float):
        # pyproj만 사용 (필수)
        x, y = self._proj(lon_deg, lat_deg)  # (lon, lat) 순서 주의
        return float(x), float(y)

    @staticmethod
    def _path_to_arrays(path_msg: Path):
        xs = np.array([ps.pose.position.x for ps in path_msg.poses], dtype=float)
        ys = np.array([ps.pose.position.y for ps in path_msg.poses], dtype=float)
        return xs, ys

    @staticmethod
    def _nearest_index(xs: np.ndarray, ys: np.ndarray, x0: float, y0: float) -> int:
        dx = xs - x0
        dy = ys - y0
        d2 = dx*dx + dy*dy
        return int(np.argmin(d2))


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
