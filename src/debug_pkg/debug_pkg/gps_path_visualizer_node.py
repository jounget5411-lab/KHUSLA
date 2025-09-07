import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Header
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from interfaces_pkg.msg import PathPlanningResult

# Odometry 기반 시각화: odom이 frame_id(기본 'map')와 동일 좌표계라고 가정

class PPResultViz(Node):
    def __init__(self):
        super().__init__('pp_result_viz')
        self.declare_parameter('sub_result', '/path_planning_result')
        self.declare_parameter('sub_path',   '/gps/centerline')
        # CHANGED: 기본 오도메트리 토픽을 현재 사용 중인 것으로 맞춤
        self.declare_parameter('sub_odom',   '/odometry/filtered')
        self.declare_parameter('frame_id',   'map')
        self.declare_parameter('ahead_pick', 5)  # (참고) 예전 접선각 방식에서 쓰던 파라미터

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )

        self._frame = self.get_parameter('frame_id').value

        self._path = None      # nav_msgs/Path (map, m)
        self._pose_xy = None   # (x,y)
        self._yaw = None       # CHANGED: 오도메트리에서 얻은 yaw 저장
        self._result = None    # PathPlanningResult

        self.sub_path   = self.create_subscription(Path, self.get_parameter('sub_path').value,     self._on_path,   qos)
        self.sub_odom   = self.create_subscription(Odometry, self.get_parameter('sub_odom').value, self._on_odom,  qos)
        self.sub_result = self.create_subscription(PathPlanningResult, self.get_parameter('sub_result').value, self._on_result, qos)

        self.pub_preview = self.create_publisher(Path, '/pp/preview_path', qos)

        self.get_logger().info("pp_result_viz ready: /pp/preview_path (map) [odom-based]")

    def _on_path(self, msg: Path):
        self._path = msg
        self._try_publish()

    def _on_odom(self, msg: Odometry):
        # 오도메트리 좌표계는 self._frame과 동일하다고 가정
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        self._pose_xy = (x, y)

        # CHANGED: 오도메트리에서 yaw을 직접 추출해 저장
        q = msg.pose.pose.orientation
        w, xq, yq, zq = float(q.w), float(q.x), float(q.y), float(q.z)
        self._yaw = math.atan2(2.0*(w*zq + xq*yq), 1.0 - 2.0*(yq*yq + zq*zq))

        self._try_publish()

    def _on_result(self, msg: PathPlanningResult):
        self._result = msg
        self._try_publish()

    def _try_publish(self):
        # CHANGED: yaw도 준비되어 있어야 정확히 역변환 가능
        if self._path is None or self._pose_xy is None or self._result is None or self._yaw is None:
            return

        # 1) 현재 위치
        x0, y0 = self._pose_xy

        # (참고) 예전 코드: centerline 접선으로 alpha를 추정했음 → 오차 가능
        # 우리는 path_planner가 사용한 것과 동일하게, 오도메트리의 yaw을 그대로 사용
        alpha = self._yaw  # CHANGED: 핵심

        # 2) 로컬(x,y) → map(x,y) 역변환
        x_local = np.array(self._result.x_points, dtype=float)
        y_local = np.array(self._result.y_points, dtype=float)
        if x_local.size < 2 or y_local.size != x_local.size:
            return

        ca = math.cos(alpha)
        sa = math.sin(alpha)

        # path_planner의 정/역변환과 정확히 짝이 되는 수식:
        #   [x_local]   [  sa  -ca ] [dx]
        #   [y_local] = [  ca   sa ] [dy]
        # 역변환(행렬 전치):
        #   [dx]   [  sa   ca ] [x_local]
        #   [dy] = [ -ca   sa ] [y_local]
        dx =  sa * x_local + ca * y_local
        dy = -ca * x_local + sa * y_local

        x_map = x0 + dx
        y_map = y0 + dy

        # 3) Path로 퍼블리시
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
