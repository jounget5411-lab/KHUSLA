# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import cv2
import random
import numpy as np
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState

import message_filters
from cv_bridge import CvBridge
from ultralytics.utils.plotting import Annotator, colors

from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from interfaces_pkg.msg import BoundingBox2D
from interfaces_pkg.msg import KeyPoint2D
from interfaces_pkg.msg import KeyPoint3D
from interfaces_pkg.msg import Detection
from interfaces_pkg.msg import DetectionArray


class YoloVisualizerNode(LifecycleNode):

    def __init__(self) -> None:
        super().__init__("yolo_visualizer_node")

        self._class_to_color = {}
        self.cv_bridge = CvBridge()

        self._last_v8 = None  # type: DetectionArray | None
        self._last_v5 = None  # type: DetectionArray | None

        # params
        self.declare_parameter("image_reliability",
                               QoSReliabilityPolicy.RELIABLE)

        self.get_logger().info("Debug node created")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring {self.get_name()}')

        self.image_qos_profile = QoSProfile(
            reliability=self.get_parameter(
                "image_reliability").get_parameter_value().integer_value,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # combined overlay publisher (v8 + v5 on one image)
        self._dbg_pub_combined = self.create_publisher(Image, "yolo_visualized_img_combined", 10)
        self._bb_markers_pub_combined = self.create_publisher(
            MarkerArray, "dbg_bb_markers_combined", 10)
        self._kp_markers_pub_combined = self.create_publisher(
            MarkerArray, "dbg_kp_markers_combined", 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Activating {self.get_name()}')

        # subs
        self.image_sub = message_filters.Subscriber(
            self, Image, "image_raw", qos_profile=self.image_qos_profile)
        self.detections_sub = message_filters.Subscriber(
            self, DetectionArray, "detections", qos_profile=10)

        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            (self.image_sub, self.detections_sub), 10, 0.5)
        self._synchronizer.registerCallback(self.detections_cb)

        # add v5 subscriber and synchronizer
        self.detections_sub_v5 = message_filters.Subscriber(
            self, DetectionArray, "detections_v5", qos_profile=10)

        self._synchronizer_v5 = message_filters.ApproximateTimeSynchronizer(
            (self.image_sub, self.detections_sub_v5), 10, 0.5)
        self._synchronizer_v5.registerCallback(self.detections_cb_v5)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating {self.get_name()}')

        self.destroy_subscription(self.image_sub.sub)
        self.destroy_subscription(self.detections_sub.sub)
        self.destroy_subscription(self.detections_sub_v5.sub)

        del self._synchronizer
        del self._synchronizer_v5

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning up {self.get_name()}')

        self.destroy_publisher(self._dbg_pub_combined)
        self.destroy_publisher(self._bb_markers_pub_combined)
        self.destroy_publisher(self._kp_markers_pub_combined)

        return TransitionCallbackReturn.SUCCESS

    def draw_box(self, cv_image: np.array, detection: Detection, color: Tuple[int]) -> np.array:

        # get detection info
        label = detection.class_name
        score = detection.score
        box_msg: BoundingBox2D = detection.bbox
        track_id = detection.id

        min_pt = (round(box_msg.center.position.x - box_msg.size.x / 2.0),
                  round(box_msg.center.position.y - box_msg.size.y / 2.0))
        max_pt = (round(box_msg.center.position.x + box_msg.size.x / 2.0),
                  round(box_msg.center.position.y + box_msg.size.y / 2.0))

        # draw box
        cv2.rectangle(cv_image, min_pt, max_pt, color, 2)

        # write text
        label = "{} ({}) ({:.3f})".format(label, str(track_id), score)
        pos = (min_pt[0] + 5, min_pt[1] + 25)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(cv_image, label, pos, font,
                    1, color, 1, cv2.LINE_AA)

        return cv_image

    def draw_mask(self, cv_image: np.array, detection: Detection, color: Tuple[int]) -> np.array:

        mask_msg = detection.mask
        mask_array = np.array([[int(ele.x), int(ele.y)]
                              for ele in mask_msg.data])

        if mask_msg.data:
            layer = cv_image.copy()
            layer = cv2.fillPoly(layer, pts=[mask_array], color=color)
            cv2.addWeighted(cv_image, 0.4, layer, 0.6, 0, cv_image)
            cv_image = cv2.polylines(cv_image, [mask_array], isClosed=True,
                                     color=color, thickness=2, lineType=cv2.LINE_AA)
        return cv_image

    def draw_keypoints(self, cv_image: np.array, detection: Detection) -> np.array:

        keypoints_msg = detection.keypoints

        ann = Annotator(cv_image)

        kp: KeyPoint2D
        for kp in keypoints_msg.data:
            color_k = [int(x) for x in ann.kpt_color[kp.id - 1]
                       ] if len(keypoints_msg.data) == 17 else colors(kp.id - 1)

            cv2.circle(cv_image, (int(kp.point.x), int(kp.point.y)),
                       5, color_k, -1, lineType=cv2.LINE_AA)

        def get_pk_pose(kp_id: int) -> Tuple[int]:
            for kp in keypoints_msg.data:
                if kp.id == kp_id:
                    return (int(kp.point.x), int(kp.point.y))
            return None

        for i, sk in enumerate(ann.skeleton):
            kp1_pos = get_pk_pose(sk[0])
            kp2_pos = get_pk_pose(sk[1])

            if kp1_pos is not None and kp2_pos is not None:
                cv2.line(cv_image, kp1_pos, kp2_pos, [
                    int(x) for x in ann.limb_color[i]], thickness=2, lineType=cv2.LINE_AA)

        return cv_image

    def create_bb_marker(self, detection: Detection, color: Tuple[int], ns: str = "yolo_combined") -> Marker:

        bbox3d = detection.bbox3d

        marker = Marker()
        marker.header.frame_id = bbox3d.frame_id

        marker.ns = ns
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.frame_locked = False

        marker.pose.position.x = bbox3d.center.position.x
        marker.pose.position.y = bbox3d.center.position.y
        marker.pose.position.z = bbox3d.center.position.z

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = bbox3d.size.x
        marker.scale.y = bbox3d.size.y
        marker.scale.z = bbox3d.size.z

        marker.color.b = color[0] / 255.0
        marker.color.g = color[1] / 255.0
        marker.color.r = color[2] / 255.0
        marker.color.a = 0.4

        marker.lifetime = Duration(seconds=0.5).to_msg()
        marker.text = detection.class_name

        return marker

    def create_kp_marker(self, keypoint: KeyPoint3D, ns: str = "yolo_combined") -> Marker:

        marker = Marker()
        marker.ns = ns
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.frame_locked = False

        marker.pose.position.x = keypoint.point.x
        marker.pose.position.y = keypoint.point.y
        marker.pose.position.z = keypoint.point.z

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.b = keypoint.score * 255.0
        marker.color.g = 0.0
        marker.color.r = (1.0 - keypoint.score) * 255.0
        marker.color.a = 0.4

        marker.lifetime = Duration(seconds=0.5).to_msg()
        marker.text = str(keypoint.id)

        return marker

    def overlay_detections(self, cv_image: np.array, detections_msg: DetectionArray, source_tag: str = "") -> np.array:
        if detections_msg is None:
            return cv_image
        for detection in detections_msg.detections:
            label = detection.class_name
            if label not in self._class_to_color:
                r = random.randint(0, 255)
                g = random.randint(0, 255)
                b = random.randint(0, 255)
                self._class_to_color[label] = (r, g, b)
            color = self._class_to_color[label]
            cv_image = self.draw_box(cv_image, detection, color)
            cv_image = self.draw_mask(cv_image, detection, color)
            cv_image = self.draw_keypoints(cv_image, detection)
            # draw small source tag near top-left of the bbox if provided
            if source_tag:
                box = detection.bbox
                min_pt = (round(box.center.position.x - box.size.x / 2.0),
                          round(box.center.position.y - box.size.y / 2.0))
                cv2.putText(cv_image, f"[{source_tag}]", (min_pt[0] + 5, max(0, min_pt[1] - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1, cv2.LINE_AA)
        return cv_image

    def detections_cb(self, img_msg: Image, detection_msg: DetectionArray) -> None:

        self._last_v8 = detection_msg

        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg)
        bb_marker_array = MarkerArray()
        kp_marker_array = MarkerArray()

        detection: Detection
        for detection in detection_msg.detections:

            # random color
            label = detection.class_name

            if label not in self._class_to_color:
                r = random.randint(0, 255)
                g = random.randint(0, 255)
                b = random.randint(0, 255)
                self._class_to_color[label] = (r, g, b)

            color = self._class_to_color[label]

            cv_image = self.draw_box(cv_image, detection, color)
            cv_image = self.draw_mask(cv_image, detection, color)
            cv_image = self.draw_keypoints(cv_image, detection)

            if detection.bbox3d.frame_id:
                marker = self.create_bb_marker(detection, color, ns="v8")
                marker.header.stamp = img_msg.header.stamp
                marker.id = len(bb_marker_array.markers)
                bb_marker_array.markers.append(marker)

            if detection.keypoints3d.frame_id:
                for kp in detection.keypoints3d.data:
                    marker = self.create_kp_marker(kp, ns="v8")
                    marker.header.frame_id = detection.keypoints3d.frame_id
                    marker.header.stamp = img_msg.header.stamp
                    marker.id = len(kp_marker_array.markers)
                    kp_marker_array.markers.append(marker)

        # Add v5 markers to combined arrays if available
        if self._last_v5 is not None:
            for det in self._last_v5.detections:
                label = det.class_name
                if label not in self._class_to_color:
                    r = random.randint(0, 255)
                    g = random.randint(0, 255)
                    b = random.randint(0, 255)
                    self._class_to_color[label] = (r, g, b)
                color = self._class_to_color[label]
                if det.bbox3d.frame_id:
                    m = self.create_bb_marker(det, color, ns="v5")
                    m.header.stamp = img_msg.header.stamp
                    m.id = len(bb_marker_array.markers)
                    bb_marker_array.markers.append(m)
                if det.keypoints3d.frame_id:
                    for kp in det.keypoints3d.data:
                        m = self.create_kp_marker(kp, ns="v5")
                        m.header.frame_id = det.keypoints3d.frame_id
                        m.header.stamp = img_msg.header.stamp
                        m.id = len(kp_marker_array.markers)
                        kp_marker_array.markers.append(m)

        # combined overlay (v8 + latest v5)
        combined = cv_image.copy()
        if self._last_v5 is not None:
            combined = self.overlay_detections(combined, self._last_v5, "v5")
        # v8 tag to distinguish
        combined = self.overlay_detections(combined, detection_msg, "v8")
        self._dbg_pub_combined.publish(self.cv_bridge.cv2_to_imgmsg(combined, encoding=img_msg.encoding))
        self._bb_markers_pub_combined.publish(bb_marker_array)
        self._kp_markers_pub_combined.publish(kp_marker_array)

    def detections_cb_v5(self, img_msg: Image, detection_msg: DetectionArray) -> None:

        self._last_v5 = detection_msg

        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg)
        bb_marker_array = MarkerArray()
        kp_marker_array = MarkerArray()

        detection: Detection
        for detection in detection_msg.detections:

            # random color
            label = detection.class_name

            if label not in self._class_to_color:
                r = random.randint(0, 255)
                g = random.randint(0, 255)
                b = random.randint(0, 255)
                self._class_to_color[label] = (r, g, b)

            color = self._class_to_color[label]

            cv_image = self.draw_box(cv_image, detection, color)
            cv_image = self.draw_mask(cv_image, detection, color)
            cv_image = self.draw_keypoints(cv_image, detection)

            if detection.bbox3d.frame_id:
                marker = self.create_bb_marker(detection, color, ns="v5")
                marker.header.stamp = img_msg.header.stamp
                marker.id = len(bb_marker_array.markers)
                bb_marker_array.markers.append(marker)

            if detection.keypoints3d.frame_id:
                for kp in detection.keypoints3d.data:
                    marker = self.create_kp_marker(kp, ns="v5")
                    marker.header.frame_id = detection.keypoints3d.frame_id
                    marker.header.stamp = img_msg.header.stamp
                    marker.id = len(kp_marker_array.markers)
                    kp_marker_array.markers.append(marker)

        # Add v8 markers to combined arrays if available
        if self._last_v8 is not None:
            for det in self._last_v8.detections:
                label = det.class_name
                if label not in self._class_to_color:
                    r = random.randint(0, 255)
                    g = random.randint(0, 255)
                    b = random.randint(0, 255)
                    self._class_to_color[label] = (r, g, b)
                color = self._class_to_color[label]
                if det.bbox3d.frame_id:
                    m = self.create_bb_marker(det, color, ns="v8")
                    m.header.stamp = img_msg.header.stamp
                    m.id = len(bb_marker_array.markers)
                    bb_marker_array.markers.append(m)
                if det.keypoints3d.frame_id:
                    for kp in det.keypoints3d.data:
                        m = self.create_kp_marker(kp, ns="v8")
                        m.header.frame_id = det.keypoints3d.frame_id
                        m.header.stamp = img_msg.header.stamp
                        m.id = len(kp_marker_array.markers)
                        kp_marker_array.markers.append(m)

        # combined overlay (v8 + v5)
        combined = cv_image.copy()
        if self._last_v8 is not None:
            combined = self.overlay_detections(combined, self._last_v8, "v8")
        combined = self.overlay_detections(combined, detection_msg, "v5")
        self._dbg_pub_combined.publish(self.cv_bridge.cv2_to_imgmsg(combined, encoding=img_msg.encoding))
        self._bb_markers_pub_combined.publish(bb_marker_array)
        self._kp_markers_pub_combined.publish(kp_marker_array)


def main():
    rclpy.init()
    node = YoloVisualizerNode()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
