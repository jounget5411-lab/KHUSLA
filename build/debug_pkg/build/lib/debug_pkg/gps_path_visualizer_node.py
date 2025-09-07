import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from interfaces_pkg.msg import PathPlanningResult
import pyproj

# path_planner_node와 동일한 원점 사용!
ORIGIN_LAT = 37.28894785
ORIGIN_LON = 127.10763105

class PPResultViz(Node):
    def __init__(self):
        super().__init__('pp_result_viz')
        self.declare_parameter('sub_result', '/path_planning_result')
        self.declare_parameter('sub_path',   '/gps/centerline')
        self.declare_parameter('sub_gps',    '/gps/fix')
        self.declare_parameter('frame_id',   'map')
        self.declare_parameter('origin_lat', ORIGIN_LAT)
        self.declare_parameter('origin_lon', ORIGIN_LON)
        self.declare_parameter('ahead_pick', 5)  # 접선 각도 계산에 쓸 오프셋

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )

        self._frame = self.get_parameter('frame_id').value
        lat0 = float(self.get_parameter('origin_lat').value)
        lon0 = float(self.get_parameter('origin_lon').value)
        self._proj = pyproj.Proj(proj='aeqd', lat_0=lat0, lon_0=lon0, datum='WGS84', units='m')

        self._path = None      # nav_msgs/Path (map, m)
        self._fix = None       # (lat,lon)
        self._result = None    # PathPlanningResult

        self.sub_path   = self.create_subscription(Path, self.get_parameter('sub_path').value,   self._on_path,   qos)
        self.sub_fix    = self.create_subscription(NavSatFix, self.get_parameter('sub_gps').value, self._on_fix,  qos)
        self.sub_result = self.create_subscription(PathPlanningResult, self.get_parameter('sub_result').value, self._on_result, qos)

        self.pub_preview = self.create_publisher(Path, '/pp/preview_path', qos)

        self.get_logger().info("pp_result_viz ready: /pp/preview_path (map)")

    def _on_path(self, msg: Path):
        self._path = msg
        self._try_publish()

    def _on_fix(self, msg: NavSatFix):
        if msg.status.status < 0:
            return
        self._fix = (float(msg.latitude), float(msg.longitude))
        self._try_publish()

    def _on_result(self, msg: PathPlanningResult):
        self._result = msg
        self._try_publish()

    def _try_publish(self):
        if self._path is None or self._fix is None or self._result is None:
            return

        # 1) 현재 위치를 map(m)으로
        lat, lon = self._fix
        x0, y0 = self._proj(lon, lat)  # (lon, lat)!

        # 2) centerline에서 현재 위치 근처의 접선 각도(theta) 구하기
        xs = np.array([ps.pose.position.x for ps in self._path.poses], dtype=float)
        ys = np.array([ps.pose.position.y for ps in self._path.poses], dtype=float)
        if xs.size < 2:
            return
        idx = int(np.argmin((xs - x0)**2 + (ys - y0)**2))
        j = min(idx + int(self.get_parameter('ahead_pick').value), xs.size - 1)
        theta = math.atan2(ys[j] - ys[idx], xs[j] - xs[idx])         # 경로 접선 각
        alpha = (math.pi / 2.0) - theta                              # path_planner의 로컬 회전각과 동일

        # 3) 로컬(x,y) → map(x,y) 역변환
        x_local = np.array(self._result.x_points, dtype=float)
        y_local = np.array(self._result.y_points, dtype=float)
        if x_local.size < 2 or y_local.size != x_local.size:
            return

        ca = math.cos(alpha); sa = math.sin(alpha)
        # [dx;dy] = R(-alpha) * [x_local;y_local]
        dx =  ca * x_local + sa * y_local
        dy = -sa * x_local + ca * y_local
        x_map = x0 + dx
        y_map = y0 + dy

        # 4) Path로 퍼블리시
        path = Path()
        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = self._frame
        path.header = hdr
        poses = []
        for xm, ym in zip(x_map, y_map):
            ps = PoseStamped()
            ps.header = hdr
            ps.pose.position.x = float(xm)
            ps.pose.position.y = float(ym)
            ps.pose.orientation.w = 1.0
            poses.append(ps)
        path.poses = poses
        self.pub_preview.publish(path)


def main():
    rclpy.init()
    node = PPResultViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()