import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Header
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from interfaces_pkg.msg import PathPlanningResult

class PPResultViz(Node):
    def __init__(self):
        super().__init__('pp_result_viz')
        self.declare_parameter('use_sim_time', True)        # ★ 추가
        self.declare_parameter('sub_result', '/path_planning_result')
        self.declare_parameter('sub_path',   '/gps/centerline')
        self.declare_parameter('sub_odom',   '/odometry/filtered_map')
        self.declare_parameter('frame_id',   'map')
        self.declare_parameter('ahead_pick', 5)

        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # ★ 변경
            depth=1,
        )

        self._frame = self.get_parameter('frame_id').value

        self._path = None
        self._pose_xy = None
        self._result = None

        self.sub_path   = self.create_subscription(Path, self.get_parameter('sub_path').value,     self._on_path,   sub_qos)
        self.sub_odom   = self.create_subscription(Odometry, self.get_parameter('sub_odom').value, self._on_odom,   sub_qos)
        self.sub_result = self.create_subscription(PathPlanningResult, self.get_parameter('sub_result').value, self._on_result, sub_qos)

        # ★ 토픽 분리: 충돌 방지 (기존 /pp/preview_path → /pp/preview_path_debug)
        self.pub_preview = self.create_publisher(Path, '/pp/preview_path_debug', pub_qos)

        self.get_logger().info("pp_result_viz ready: /pp/preview_path_debug (map) [odom-based]")

    def _on_path(self, msg: Path):
        self._path = msg
        self._try_publish()

    def _on_odom(self, msg: Odometry):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        self._pose_xy = (x, y)
        self._try_publish()

    def _on_result(self, msg: PathPlanningResult):
        self._result = msg
        self._try_publish()

    def _try_publish(self):
        if self._path is None or self._pose_xy is None or self._result is None:
            return

        x0, y0 = self._pose_xy

        xs = np.array([ps.pose.position.x for ps in self._path.poses], dtype=float)
        ys = np.array([ps.pose.position.y for ps in self._path.poses], dtype=float)
        if xs.size < 2:
            return
        idx = int(np.argmin((xs - x0)**2 + (ys - y0)**2))
        j = min(idx + int(self.get_parameter('ahead_pick').value), xs.size - 1)
        theta = math.atan2(ys[j] - ys[idx], xs[j] - xs[idx])
        alpha = (math.pi / 2.0) - theta  # path_planner의 로컬 정의에 맞춤

        x_local = np.array(self._result.x_points, dtype=float)
        y_local = np.array(self._result.y_points, dtype=float)
        if x_local.size < 2 or y_local.size != x_local.size:
            return

        ca = math.cos(alpha); sa = math.sin(alpha)
        dx =  ca * x_local + sa * y_local
        dy = -sa * x_local + ca * y_local
        x_map = x0 + dx
        y_map = y0 + dy

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
