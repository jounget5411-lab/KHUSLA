#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

class PoseVToModelPose(Node):
    def __init__(self):
        super().__init__('posev_to_model_pose')
        self.declare_parameter('model_name', 'henes_t870')
        self.declare_parameter('out_topic', '/henes_t870/world_pose')
        self.model = self.get_parameter('model_name').value
        self.pub = self.create_publisher(Pose, self.get_parameter('out_topic').value, 10)
        self.sub = self.create_subscription(TFMessage, '/gz_pose_info', self.cb, 10)
        self.get_logger().info(f'Filtering Pose_V for model="{self.model}"')

    def cb(self, msg: TFMessage):
        # TFMessage.transforms: each has .header.frame_id (parent) and .child_frame_id (entity name)
        for t in msg.transforms:
            name = t.child_frame_id
            if not name:
                continue
            # 다양한 네이밍에 대응: 정확히 일치 or 끝에 맞춤 or 포함
            if name == self.model or name.endswith(self.model) or self.model in name:
                p = Pose()
                p.position.x = t.transform.translation.x
                p.position.y = t.transform.translation.y
                p.position.z = t.transform.translation.z
                p.orientation = t.transform.rotation
                self.pub.publish(p)
                break

def main():
    rclpy.init()
    rclpy.spin(PoseVToModelPose())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

