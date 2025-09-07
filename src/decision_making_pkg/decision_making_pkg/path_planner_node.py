import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Path, Odometry
from interfaces_pkg.msg import PathPlanningResult

# 기본 입력 토픽 이름을 EKF의 최종 결과물인 /odometry/filtered_map으로 설정합니다.
SUB_PATH_TOPIC_NAME = "/gps/centerline"
SUB_ODOM_TOPIC_NAME = "/odometry/filtered_map" # <-- EKF의 최종 결과물을 사용
PUB_TOPIC_NAME      = "/path_planning_result"

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        # 파라미터 선언
        self.sub_path_topic = self.declare_parameter('sub_path_topic', SUB_PATH_TOPIC_NAME).value
        self.sub_odom_topic = self.declare_parameter('sub_odom_topic', SUB_ODOM_TOPIC_NAME).value
        self.pub_topic      = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        self.ahead_len = int(self.declare_parameter('ahead_len', 50).value)
        self.stride    = int(self.declare_parameter('downsample_stride', 1).value)
        self.delay_preview_sec = float(self.declare_parameter('delay_preview_sec', 0.10).value)

        # QoS 설정
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )

        # 퍼블리셔 및 서브스크라이버
        self.publisher = self.create_publisher(PathPlanningResult, self.pub_topic, self.qos_profile)
        self.path_sub  = self.create_subscription(Path,     self.sub_path_topic, self._on_path, self.qos_profile)
        self.odom_sub  = self.create_subscription(Odometry, self.sub_odom_topic, self._on_odom, self.qos_profile)

        # 내부 상태 변수
        self._last_path = None
        self._have_odom = False
        self._px, self._py, self._yaw = 0.0, 0.0, 0.0
        self._v, self._omega = 0.0, 0.0
        self._last_idx = 0

        self.get_logger().info(f"Path planner ready. Subscribing to Odometry: '{self.sub_odom_topic}'")

    def _on_path(self, msg: Path):
        # [디버그 로그 추가] 경로 데이터가 들어왔는지 확인
        self.get_logger().debug(f"Received new global path with {len(msg.poses)} points.")
        if not msg.poses:
            return
        self._last_path = msg
        self._try_publish()

    def _on_odom(self, msg: Odometry):
        # [디버그 로그 추가] Odom 데이터가 들어왔는지 확인
        self.get_logger().debug(f"Received new odometry data. Frame: {msg.header.frame_id}")
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._px = float(p.x)
        self._py = float(p.y)
        w, x, y, z = float(q.w), float(q.x), float(q.y), float(q.z)
        self._yaw = math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))
        self._v = float(getattr(msg.twist.twist.linear, 'x', 0.0))
        self._omega = float(getattr(msg.twist.twist.angular, 'z', 0.0))
        self._have_odom = True
        self._try_publish()

    def _try_publish(self):
        if (self._last_path is None) or (not self._have_odom):
            # [디버그 로그 추가] 입력 데이터가 부족하여 건너뛰는지 확인
            self.get_logger().warn("Skipping publish: global path or odometry is missing.", throttle_duration_sec=5.0)
            return

        T = max(0.0, self.delay_preview_sec)
        px = self._px + self._v * T * math.cos(self._yaw)
        py = self._py + self._v * T * math.sin(self._yaw)
        yaw = self._yaw + self._omega * T

        xs, ys = self._path_to_arrays(self._last_path)
        if xs.size < 2: return

        idx = self._nearest_index(xs, ys, px, py)
        if idx < self._last_idx:
            idx = self._last_idx
        self._last_idx = idx

        start = idx
        end   = min(len(xs), start + self.ahead_len)
        xs_seg = xs[start:end:self.stride]
        ys_seg = ys[start:end:self.stride]

        # [디버그 로그 추가] 경로 자르기 결과 확인
        if xs_seg.size < 2:
            self.get_logger().warn(f"Skipping publish: Not enough forward points after slicing. Found {xs_seg.size} points.", throttle_duration_sec=5.0)
            return

        dx = xs_seg - px
        dy = ys_seg - py
        cy, sy = math.cos(yaw), math.sin(yaw)
        x_local =  sy * dx - cy * dy
        y_local =  cy * dx + sy * dy

        out = PathPlanningResult()
        out.x_points = [float(v) for v in x_local]
        out.y_points = [float(v) for v in y_local]
        self.publisher.publish(out)

        d_near = math.hypot(xs[idx]-px, ys[idx]-py)
        self.get_logger().info(
            f"Published {len(out.x_points)} pts (nearest_idx={idx}, d_near={d_near:.2f} m, current_pos=({self._px:.2f}, {self._py:.2f}))"
            , throttle_duration_sec=1.0)

    # ... (나머지 함수는 그대로) ...
    @staticmethod
    def _path_to_arrays(path_msg: Path):
        xs = np.array([ps.pose.position.x for ps in path_msg.poses], dtype=float)
        ys = np.array([ps.pose.position.y for ps in path_msg.poses], dtype=float)
        return xs, ys

    @staticmethod
    def _nearest_index(xs: np.ndarray, ys: np.ndarray, x0: float, y0: float) -> int:
        dx = xs - x0
        dy = ys - dy
        return int(np.argmin(dx*dx + dy*dy))

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
