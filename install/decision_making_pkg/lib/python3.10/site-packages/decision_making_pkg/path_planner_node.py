import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
)

from nav_msgs.msg import Path, Odometry
from interfaces_pkg.msg import PathPlanningResult

# --------------- Parameters (defaults) ---------------
SUB_PATH_TOPIC_NAME = "/gps/centerline"        # gps_centerline_node가 퍼블리시하는 Path (frame: map, 단위: m)
SUB_ODOM_TOPIC_NAME = "/odometry/filtered_map" # EKF 결과(Odometry, frame: map, 단위: m)
PUB_TOPIC_NAME      = "/path_planning_result"  # motion_planner가 구독하는 기존 토픽/메시지 유지


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        # ---- 파라미터 선언 ----
        self.sub_path_topic = self.declare_parameter('sub_path_topic', SUB_PATH_TOPIC_NAME).value
        self.sub_odom_topic = self.declare_parameter('sub_odom_topic', SUB_ODOM_TOPIC_NAME).value
        self.pub_topic      = self.declare_parameter('pub_topic',      PUB_TOPIC_NAME).value

        # 앞으로 사용할 경로 길이/간격 제어
        self.ahead_len = int(self.declare_parameter('ahead_len', 50).value)         # 앞쪽으로 보낼 점 개수
        self.stride    = int(self.declare_parameter('downsample_stride', 1).value)  # 다운샘플 간격(1=그대로)

        # 지연 보상(프리뷰) 시간 [s] (Odometry의 twist를 이용하여 미래 자세 예측)
        self.delay_preview_sec = float(self.declare_parameter('delay_preview_sec', 0.10).value)

        # ---- QoS ----
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )

        # ---- 퍼블리셔/서브스크라이버 ----
        self.publisher = self.create_publisher(PathPlanningResult, self.pub_topic, self.qos_profile)
        self.path_sub  = self.create_subscription(Path,     self.sub_path_topic, self._on_path, self.qos_profile)
        self.odom_sub  = self.create_subscription(Odometry, self.sub_odom_topic, self._on_odom, self.qos_profile)

        # ---- 내부 상태 ----
        self._last_path = None     # nav_msgs/Path
        self._have_odom = False
        self._px = 0.0
        self._py = 0.0
        self._yaw = 0.0
        self._v = 0.0
        self._omega = 0.0

        self.get_logger().info(
            f"path_planner_node ready: sub_path='{self.sub_path_topic}', sub_odom='{self.sub_odom_topic}', "
            f"pub='{self.pub_topic}', ahead_len={self.ahead_len}, stride={self.stride}, preview={self.delay_preview_sec}s, frame='map'"
        )

    # ---------------- Callbacks ----------------
    def _on_path(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn("Received empty Path; skip")
            return
        self._last_path = msg
        self._try_publish()

    def _on_odom(self, msg: Odometry):
        # pose
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._px = float(p.x)
        self._py = float(p.y)
        # quaternion -> yaw (Z) without tf_transformations
        w, x, y, z = float(q.w), float(q.x), float(q.y), float(q.z)
        self._yaw = math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))

        # twist (있으면 사용, 없으면 0으로 처리)
        self._v = float(getattr(msg.twist.twist.linear, 'x', 0.0))
        self._omega = float(getattr(msg.twist.twist.angular, 'z', 0.0))

        self._have_odom = True
        self._try_publish()

    # --------------- Core logic ---------------
    def _try_publish(self):
        if (self._last_path is None) or (not self._have_odom):
            return

        # 1) 현재(또는 프리뷰) 자세 계산
        T = max(0.0, self.delay_preview_sec)
        px = self._px + self._v * T * math.cos(self._yaw)
        py = self._py + self._v * T * math.sin(self._yaw)
        yaw = self._yaw + self._omega * T

        # 2) Path를 배열로 추출
        xs, ys = self._path_to_arrays(self._last_path)
        if xs.size < 2:
            self.get_logger().warn("Path has <2 points; skip")
            return

        # 3) 최근접점 탐색(맵 프레임)
        idx = self._nearest_index(xs, ys, px, py)

        # 4) 앞 구간 선택 + 다운샘플
        start = idx
        end   = min(len(xs), start + max(2, self.ahead_len * max(1, self.stride)))
        xs_seg = xs[start:end:self.stride]
        ys_seg = ys[start:end:self.stride]
        if xs_seg.size < 2:
            self.get_logger().warn("Too few forward points after slicing; skip")
            return

        # 5) map -> vehicle-local 변환 (x=오른쪽, y=전방)
        #    회전행렬: [ [ sin(yaw), -cos(yaw) ],
        #               [ cos(yaw),  sin(yaw) ] ]  with (dx,dy) = (map - vehicle)
        dx = xs_seg - px
        dy = ys_seg - py
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        x_local =  sy * dx - cy * dy  # 오른쪽(+x)
        y_local =  cy * dx + sy * dy  # 전방(+y)

        # y가 증가하는 방향으로 정렬 보장(혹시 역방향이면 뒤집기)
        if y_local[-1] < y_local[0]:
            x_local = x_local[::-1]
            y_local = y_local[::-1]

        # 6) motion_planner가 기대하는 형식으로 퍼블리시
        out = PathPlanningResult()
        out.x_points = [float(v) for v in x_local]
        out.y_points = [float(v) for v in y_local]
        self.publisher.publish(out)

        self.get_logger().info(
            f"Published PathPlanningResult: {len(out.x_points)} pts (nearest_idx={idx}, yaw={yaw:.3f} rad, preview={T:.3f}s)"
        )

    # --------------- Utilities ---------------
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