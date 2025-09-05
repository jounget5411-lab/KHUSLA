# right_offset_centerline_node.py
import csv
import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import numpy as np
import pyproj

# === Hardcoded WGS84 origin (must match your path planner) ===
ORIGIN_LAT = 37.28894785
ORIGIN_LON = 127.10763105


def load_csv_latlon(path: str, has_header: bool, lat_idx: int = 6, lon_idx: int = 7) -> np.ndarray:
    """CSV에서 (lat, lon)을 읽어 (N,2) float array로 반환.
    - has_header=True면 첫 줄을 건너뜀
    - 빈 값 / 'nan' / 'inf'는 스킵
    """
    rows: List[Tuple[float, float]] = []
    with open(path, 'r', newline='') as f:
        reader = csv.reader(f)
        if has_header:
            next(reader, None)
        for row in reader:
            if len(row) <= max(lat_idx, lon_idx):
                continue
            a_str = row[lat_idx].strip()
            b_str = row[lon_idx].strip()
            if a_str == '' or b_str == '':
                continue
            if a_str.lower() in ('nan', 'inf', '-inf') or b_str.lower() in ('nan', 'inf', '-inf'):
                continue
            try:
                a = float(a_str); b = float(b_str)
            except ValueError:
                continue
            if not (math.isfinite(a) and math.isfinite(b)):
                continue
            rows.append((a, b))
    if not rows:
        raise RuntimeError(f"CSV empty or invalid (no valid lat/lon rows): {path}")
    return np.asarray(rows, dtype=float)


def latlon_to_xy_m(latlon: np.ndarray,
                   origin_ll: Tuple[float, float]) -> np.ndarray:
    """WGS84 lat/lon (deg) -> local meters (x East, y North) w.r.t origin."""
    lat0, lon0 = origin_ll
    lat = latlon[:, 0]
    lon = latlon[:, 1]
    # 지역 투영: origin 중심 AEQD
    ae_proj = pyproj.Proj(proj='aeqd', lat_0=lat0, lon_0=lon0, datum='WGS84', units='m')
    x, y = ae_proj(lon, lat)  # pyproj는 (lon, lat) 순서
    return np.vstack([x, y]).T


def ensure_forward_order(poly: np.ndarray) -> np.ndarray:
    """아크길이가 증가하는 방향이 되도록 간단히 뒤집힘만 보정."""
    if poly.shape[0] < 3:
        return poly
    head = np.linalg.norm(poly[10] - poly[0]) if poly.shape[0] > 10 else np.linalg.norm(poly[-1] - poly[0])
    tail = np.linalg.norm(poly[-1] - poly[-11]) if poly.shape[0] > 10 else np.linalg.norm(poly[-1] - poly[0])
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(poly, axis=0), axis=1))]
    if s[-1] <= 0:
        return poly
    if tail < head:
        return poly[::-1].copy()
    return poly


def resample_by_arclength_with_S(xy: np.ndarray, spacing: float) -> Tuple[np.ndarray, np.ndarray]:
    """등간격 리샘플 (간격=spacing). S는 누적거리(m)."""
    if xy.shape[0] < 2:
        return np.array([0.0], dtype=float), xy.copy()
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    if not np.isfinite(seg).all():
        seg = np.nan_to_num(seg, nan=0.0, posinf=0.0, neginf=0.0)
    s = np.r_[0.0, np.cumsum(seg)]
    if not np.isfinite(s[-1]) or s[-1] <= max(spacing, 1e-6):
        return np.array([0.0], dtype=float), xy.copy()
    S = np.arange(0.0, float(s[-1]) + 1e-6, float(spacing))
    x = np.interp(S, s, xy[:, 0])
    y = np.interp(S, s, xy[:, 1])
    return S, np.vstack([x, y]).T


def resample_by_arclength(xy: np.ndarray, spacing: float) -> np.ndarray:
    S, xy2 = resample_by_arclength_with_S(xy, spacing)
    return xy2


def offset_from_right_lane(Rm: np.ndarray, spacing: float, offset_m: float, logger=None) -> np.ndarray:
    """오른쪽 차선(Rm)으로부터 진행방향 기준 좌측으로 offset_m 평행이동."""
    if Rm.shape[0] < 2:
        raise RuntimeError("Right lane has too few points.")

    # 1) 등간격화 → 수치 안정
    S, Rr = resample_by_arclength_with_S(Rm, spacing)
    if Rr.shape[0] < 2:
        raise RuntimeError("Right lane resampling failed or too short.")

    # 2) 접선(중앙차분), 끝점은 전/후진 차분
    t = np.zeros_like(Rr)
    t[1:-1] = Rr[2:] - Rr[:-2]
    t[0]     = Rr[1] - Rr[0]
    t[-1]    = Rr[-1] - Rr[-2]

    # 정상화
    nrm = np.linalg.norm(t, axis=1, keepdims=True)
    nrm[nrm == 0.0] = 1.0
    t = t / nrm

    # 3) 좌측 법선 (-ty, tx)  (ENU에서 진행방향 좌측은 +90° 회전)
    n_left = np.column_stack((-t[:, 1], t[:, 0]))

    # 4) 오프셋 적용
    C = Rr + float(offset_m) * n_left

    # NaN/Inf 제거
    C = C[np.isfinite(C).all(axis=1)]
    if C.shape[0] < 2:
        raise RuntimeError("Offset result too short or invalid after filtering.")

    if logger is not None:
        logger.info(f"right→left offset: {offset_m} m, points={C.shape[0]}")
    return C


class RightOffsetCenterlineServer(Node):
    """
    입력: right_lane.csv (위경도 or m)
    처리: (선택) 투영 → 등간격화 → 좌측 법선 기반 offset → 마지막 등간격 스무딩
    출력: /gps/centerline (nav_msgs/Path, frame=map, 단위 m)
    """
    def __init__(self):
        super().__init__('right_offset_centerline_server')

        # 파라미터
        self.declare_parameter('right_csv', '/home/daehyeon/ros2_ws/right_lane.csv')
        self.declare_parameter('has_header', True)
        self.declare_parameter('latlon_input', True)              # 입력이 lat/lon 인가?
        self.declare_parameter('lat_idx', 6)                      # 위도 열 인덱스(0-based)
        self.declare_parameter('lon_idx', 7)                      # 경도 열 인덱스(0-based)

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('spacing_m', 0.30)                 # 등간격 리샘플 간격
        self.declare_parameter('publish_rate_hz', 2.0)            # Path 재퍼블리시 Hz
        self.declare_parameter('topic_centerline', '/gps/centerline')

        self.declare_parameter('offset_m', 1.5)                   # +면 좌측, -면 우측으로 평행이동
        self.declare_parameter('origin_lat', ORIGIN_LAT)          # AEQD 원점(경로/플래너와 동일)
        self.declare_parameter('origin_lon', ORIGIN_LON)

        # QoS
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.topic_centerline = self.get_parameter('topic_centerline').value
        self.pub_path = self.create_publisher(Path, self.topic_centerline, qos)

        # 파라미터 로드
        right_path = self.get_parameter('right_csv').value
        has_header = bool(self.get_parameter('has_header').value)
        latlon_input = bool(self.get_parameter('latlon_input').value)
        lat_idx = int(self.get_parameter('lat_idx').value)
        lon_idx = int(self.get_parameter('lon_idx').value)

        spacing = float(self.get_parameter('spacing_m').value)
        offset_m = float(self.get_parameter('offset_m').value)
        frame_id = self.get_parameter('frame_id').value
        rate_hz = float(self.get_parameter('publish_rate_hz').value)

        lat0 = float(self.get_parameter('origin_lat').value)
        lon0 = float(self.get_parameter('origin_lon').value)

        # 데이터 로드
        R = load_csv_latlon(right_path, has_header, lat_idx=lat_idx, lon_idx=lon_idx)
        self.get_logger().info(f"Loaded right CSV: {R.shape[0]} rows (lat_idx={lat_idx}, lon_idx={lon_idx})")

        # 좌표 변환
        if latlon_input:
            self.get_logger().info(f"Using hardcoded origin (lat,lon)=({lat0}, {lon0})")
            Rm = latlon_to_xy_m(R, (lat0, lon0))
        else:
            Rm = R.copy()

        # 최소 길이 확인 및 진행방향 정렬
        if Rm.shape[0] < 2:
            raise RuntimeError("Too few points in right lane after load/convert.")
        Rm = ensure_forward_order(Rm)

        # 오프셋 경로 생성
        try:
            C = offset_from_right_lane(Rm, spacing, offset_m, logger=self.get_logger())
        except Exception as e:
            raise RuntimeError(f"Right-offset centerline failed: {e}")

        # 마지막 등간격 스무딩(숫자 안정)
        C = resample_by_arclength(C, spacing)
        C = C[np.isfinite(C).all(axis=1)]
        if C.shape[0] < 2:
            raise RuntimeError("Final centerline too short/invalid after smoothing.")

        # Path 메시지 준비 및 타이머 퍼블리시
        self.path_msg = self._to_path_msg(C, frame_id)
        self.timer = self.create_timer(1.0 / max(rate_hz, 0.1), self._on_timer)

        self.get_logger().info(f"Centerline ready: {C.shape[0]} pts, spacing≈{spacing} m, "
                               f"offset={offset_m} m, frame='{frame_id}'")
        self.get_logger().info(f"Publishing on '{self.topic_centerline}' at {rate_hz} Hz")

    def _to_path_msg(self, xy: np.ndarray, frame_id: str) -> Path:
        msg = Path()
        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = frame_id
        msg.header = hdr
        poses: List[PoseStamped] = []
        for (x, y) in xy:
            ps = PoseStamped()
            ps.header = hdr
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            poses.append(ps)
        msg.poses = poses
        return msg

    def _on_timer(self):
        now = self.get_clock().now().to_msg()
        self.path_msg.header.stamp = now
        for ps in self.path_msg.poses:
            ps.header.stamp = now
        self.pub_path.publish(self.path_msg)


def main():
    rclpy.init()
    node = RightOffsetCenterlineServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()