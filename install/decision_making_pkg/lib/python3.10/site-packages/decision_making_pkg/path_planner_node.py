#!/usr/bin/env python3
import math
import numpy as np
import pyproj

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
)

from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from interfaces_pkg.msg import PathPlanningResult

# --------------- Defaults ---------------
SUB_PATH_TOPIC_NAME = "/gps/centerline"      # gps_centerline_node가 퍼블리시하는 Path (frame: map, 단위 m)
SUB_GPS_TOPIC_NAME  = "/ublox_gps_node/fix"  # NavSatFix
PUB_TOPIC_NAME      = "/path_planning_result"

# 하드코딩된 투영 기준(경로 CSV 생성 시와 동일해야 함)
ORIGIN_LAT = 37.28894785
ORIGIN_LON = 127.10763105


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        # 파라미터
        self.sub_path_topic = self.declare_parameter('sub_path_topic', SUB_PATH_TOPIC_NAME).value
        self.sub_gps_topic  = self.declare_parameter('sub_gps_topic',  SUB_GPS_TOPIC_NAME).value
        self.pub_topic      = self.declare_parameter('pub_topic',      PUB_TOPIC_NAME).value

        self.origin_lat = ORIGIN_LAT
        self.origin_lon = ORIGIN_LON

        self.ahead_len = int(self.declare_parameter('ahead_len', 50).value)
        self.stride    = int(self.declare_parameter('downsample_stride', 1).value)

        # QoS
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )

        # Pub/Sub
        self.publisher = self.create_publisher(PathPlanningResult, self.pub_topic, self.qos_profile)
        self.path_sub  = self.create_subscription(Path,      self.sub_path_topic, self._on_path, self.qos_profile)
        self.gps_sub   = self.create_subscription(NavSatFix, self.sub_gps_topic,  self._on_fix,  self.qos_profile)

        # 상태
        self._last_path = None
        self._have_fix  = False
        self._fix_lat   = None
        self._fix_lon   = None

        # 투영기
        self._proj = self._make_proj(self.origin_lat, self.origin_lon)

        # 차량 yaw 추정(LLA 연속 2점 기반)
        self._prev_xy   = None
        self._veh_yaw   = None   # [rad] map 프레임 기준
        self._yaw_alpha = 0.3    # 저역통과 게인(0~1)

        self.get_logger().info(
            f"path_planner_node ready: sub_path='{self.sub_path_topic}', sub_gps='{self.sub_gps_topic}', "
            f"pub='{self.pub_topic}', origin=({self.origin_lat}, {self.origin_lon}), "
            f"ahead_len={self.ahead_len}, stride={self.stride}, proj=AEQD"
        )

    # ---------------- Callbacks ----------------
    def _on_path(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn("Received empty Path; skip")
            return
        self._last_path = msg
        self._try_publish()

    def _on_fix(self, msg: NavSatFix):
        # 무효 GPS는 무시
        if msg.status.status < 0:
            self._have_fix = False
            return

        self._fix_lat = float(msg.latitude)
        self._fix_lon = float(msg.longitude)
        self._have_fix = True

        # 차량 yaw 추정(gps 2점)
        x_now, y_now = self._latlon_to_xy(self._fix_lat, self._fix_lon)
        if self._prev_xy is not None:
            dx = x_now - self._prev_xy[0]
            dy = y_now - self._prev_xy[1]
            if abs(dx) + abs(dy) > 1e-3:
                yaw_new = math.atan2(dy, dx)  # map 프레임(+x,+y) 기준
                if self._veh_yaw is None:
                    self._veh_yaw = yaw_new
                else:
                    dyaw = self._wrap_pi(yaw_new - self._veh_yaw)
                    self._veh_yaw = self._wrap_pi(self._veh_yaw + self._yaw_alpha * dyaw)
        self._prev_xy = (x_now, y_now)

        self._try_publish()

    # --------------- Core ---------------
    def _try_publish(self):
        if (self._last_path is None) or (not self._have_fix):
            return

        # 1) 현재 위치 map 좌표
        x0, y0 = self._latlon_to_xy(self._fix_lat, self._fix_lon)

        # 2) Path 배열화
        xs, ys = self._path_to_arrays(self._last_path)
        if xs.size < 2:
            self.get_logger().warn("Path has <2 points; skip")
            return

        # 3) 최근접 인덱스
        idx = self._nearest_index(xs, ys, x0, y0)

        # 4) 앞 구간 선택(+다운샘플)
        start = idx
        end   = min(len(xs), start + max(2, self.ahead_len * max(1, self.stride)))
        xs_seg = xs[start:end:self.stride]
        ys_seg = ys[start:end:self.stride]
        if xs_seg.size < 2:
            self.get_logger().warn("Too few forward points after slicing; skip")
            return

        # 5) map→vehicle(전방=+y, 우측=+x) 회전
        if self._veh_yaw is not None:
            yaw_used = self._veh_yaw   # 차량 진행방향
            yaw_src = "veh"
        else:
            # 초기 fallback: 경로 접선
            j = min(start + 5, len(xs) - 1)
            yaw_used = math.atan2(ys[j] - ys[start], xs[j] - xs[start])
            yaw_src = "path"

        # vehicle 프레임으로의 회전각: alpha = (pi/2) - yaw
        alpha = (math.pi / 2.0) - yaw_used
        ca, sa = math.cos(alpha), math.sin(alpha)

        dx = xs_seg - x0
        dy = ys_seg - y0
        x_local = ca * dx - sa * dy  # 차량 우측 +
        y_local = sa * dx + ca * dy  # 차량 전방 +

        # 전방 증가 정렬 보장
        if y_local[-1] < y_local[0]:
            x_local = x_local[::-1]
            y_local = y_local[::-1]

        # 6) 퍼블리시
        out = PathPlanningResult()
        out.x_points = [float(v) for v in x_local]
        out.y_points = [float(v) for v in y_local]
        self.publisher.publish(out)

        self.get_logger().info(
            f"[{yaw_src}] Published PathPlanningResult: {len(out.x_points)} pts "
            f"(nearest_idx={idx}, alpha={self._wrap_pi(alpha):.3f} rad, veh_yaw={math.degrees(yaw_used):.1f}°)"
        )

    # --------------- Utils ---------------
    def _make_proj(self, lat0: float, lon0: float):
        return pyproj.Proj(proj='aeqd', lat_0=lat0, lon_0=lon0, datum='WGS84', units='m')

    def _latlon_to_xy(self, lat_deg: float, lon_deg: float):
        x, y = self._proj(lon_deg, lat_deg)  # (lon, lat) 순서
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

    @staticmethod
    def _wrap_pi(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
