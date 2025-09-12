#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from interfaces_pkg.msg import PathPlanningResult
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

class PPPreviewViz(Node):
    def __init__(self):
        super().__init__('pp_preview_viz')
        # 기존
        # self.declare_parameter('use_sim_time', True)
        
        # 수정: 이미 선언돼 있으면 재선언하지 않음
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        # ★ QoS: publisher는 TRANSIENT_LOCAL로 (RViz 늦게 붙어도 마지막 메시지 보존)
        self.sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # ★ 변경
            depth=1,
        )

        self.fixed_frame = self.declare_parameter('fixed_frame', 'map').value
        self.base_frame  = self.declare_parameter('base_frame',  'base_footprint').value

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        self.sub = self.create_subscription(PathPlanningResult, '/path_planning_result', self.cb, self.sub_qos)
        self.pub = self.create_publisher(Path, '/pp/preview_path', self.pub_qos)
        self.get_logger().info(f'pp_preview_viz: drawing in {self.fixed_frame} using TF {self.fixed_frame} <- {self.base_frame}')

    def cb(self, msg: PathPlanningResult):
        try:
            # ★ 짧은 타임아웃을 줘서 초기 TF 지연에 견고
            tf = self.tf_buf.lookup_transform(self.fixed_frame, self.base_frame,
                                              rclpy.time.Time(), timeout=Duration(seconds=0.5))
        except Exception as e:
            self.get_logger().warn(f'no TF {self.fixed_frame}<-{self.base_frame}: {e}')
            return

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q  = tf.transform.rotation
        w, x, y, z = q.w, q.x, q.y, q.z
        yaw = math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))
        cy, sy = math.cos(yaw), math.sin(yaw)

        path = Path()
        path.header.frame_id = self.fixed_frame
        # TF의 시간으로 동기화
        path.header.stamp = tf.header.stamp

        for xl, yl in zip(msg.x_points, msg.y_points):
            X = tx +  sy*xl + cy*yl
            Y = ty +  cy*xl - sy*yl
            ps = PoseStamped()
            ps.header.frame_id = self.fixed_frame
            ps.header.stamp = path.header.stamp
            ps.pose.position.x = float(X)
            ps.pose.position.y = float(Y)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        self.pub.publish(path)

def main():
    rclpy.init()
    node = PPPreviewViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
