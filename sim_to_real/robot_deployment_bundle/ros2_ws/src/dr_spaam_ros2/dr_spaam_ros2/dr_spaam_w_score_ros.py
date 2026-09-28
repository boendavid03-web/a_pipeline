#!/usr/bin/env python

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import Point, Pose, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
import time
import csv
import math
import struct

from dr_spaam.detector import Detector
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data

# ===== MODIFIED / NEW LINES =====
# TF libraries for looking up and applying transforms
from tf2_ros import Buffer, TransformListener, TransformException

class DrSpaamROS(Node):
    """ROS 2 node to detect pedestrians using DROW3 or DR-SPAAM, 
    with post-processing to combine close detections."""

    def __init__(self):
        super().__init__('dr_spaam_ros')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('weight_file', '/default/path/to/weight.pth'),
                ('conf_thresh', 0.9),
                ('stride', 1),
                ('detector_model', 'DR-SPAAM'),
                ('panoramic_scan', True),
                ('reverse_scan', False),
                ('drow_to_ros', False),
                ('target_frame', 'odom'),
                ('device', 'auto'),
                ('publisher.detections.topic', '/dr_spaam_detections'),
                ('publisher.detections.queue_size', 10),
                ('publisher.detections.latch', False),
                ('publisher.rviz.topic', '/dr_spaam_rviz'),
                ('publisher.rviz.queue_size', 10),
                ('publisher.rviz.latch', False),
                ('publisher.scored.topic', '/dr_spaam_detections_scored'),
                ('publisher.scored.queue_size', 10),
                ('subscriber.scan.topic', '/scan'),
                ('subscriber.scan.queue_size', 10),
            ]
        )
        self._read_params()

        self._detector = Detector(
            self.weight_file,
            model=self.detector_model,
            gpu=(str(self.get_parameter('device').value) == 'cuda' or
                 (str(self.get_parameter('device').value) == 'auto' and __import__('torch').cuda.is_available())),
            stride=self.stride,
            panoramic_scan=self.panoramic_scan,
        )
        self._init()

        # ===== MODIFIED / NEW LINES =====
        # Create a tf2 buffer + listener for transforms
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def _read_params(self):
        """Reads parameters from node."""
        self.weight_file = self.get_parameter('weight_file').value
        self.conf_thresh = self.get_parameter('conf_thresh').value
        self.stride = self.get_parameter('stride').value
        self.detector_model = self.get_parameter('detector_model').value
        self.panoramic_scan = self.get_parameter('panoramic_scan').value
        self.reverse_scan = self.get_parameter('reverse_scan').value
        self.drow_to_ros = self.get_parameter('drow_to_ros').value
        self.target_frame = self.get_parameter('target_frame').value

    def _init(self):
        """Initializes publishers and subscribers."""
        # Publisher for detections
        det_topic = self.get_parameter('publisher.detections.topic').value
        det_queue_size = self.get_parameter('publisher.detections.queue_size').value
        det_latch = self.get_parameter('publisher.detections.latch').value

        det_qos = QoSProfile(depth=det_queue_size)
        if det_latch:
            det_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._dets_pub = self.create_publisher(PoseArray, det_topic, det_qos)

        # Publisher for RViz markers
        rviz_topic = self.get_parameter('publisher.rviz.topic').value
        rviz_queue_size = self.get_parameter('publisher.rviz.queue_size').value
        rviz_latch = self.get_parameter('publisher.rviz.latch').value

        rviz_qos = QoSProfile(depth=rviz_queue_size)
        if rviz_latch:
            rviz_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._rviz_pub = self.create_publisher(MarkerArray, rviz_topic, rviz_qos)

        scored_topic = self.get_parameter('publisher.scored.topic').value
        scored_queue_size = self.get_parameter('publisher.scored.queue_size').value
        self._scored_pub = self.create_publisher(
            PointCloud2, scored_topic, QoSProfile(depth=scored_queue_size)
        )

        # Subscriber for laser scans
        scan_topic = self.get_parameter('subscriber.scan.topic').value
        self._scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._scan_callback, qos_profile_sensor_data
        )

    def _scan_callback(self, msg):
        if self._dets_pub.get_subscription_count() == 0 \
           and self._rviz_pub.get_subscription_count() == 0 \
           and self._scored_pub.get_subscription_count() == 0:
            return

        if not self._detector.is_ready():
            self._detector.set_laser_fov(np.rad2deg(msg.angle_max - msg.angle_min))

        scan = np.asarray(msg.ranges, dtype=np.float32)
        invalid = (
            ~np.isfinite(scan)
            | (scan < max(0.0, float(msg.range_min)))
            | (scan > float(msg.range_max))
        )
        scan[invalid] = 29.99
        scan = np.minimum(scan, 29.99)
        if self.reverse_scan:
            scan = np.ascontiguousarray(scan[::-1])

        # calculate the inference time and then store it to a csv file
        start = time.time()
        dets_xy, dets_cls, _ = self._detector(scan)
        end = time.time()
        inference_time = end - start
        self.get_logger().info(f"Dr_spaam inference time: {inference_time} s")



        # Apply confidence threshold
        conf_mask = (dets_cls >= self.conf_thresh).reshape(-1)
        dets_xy = dets_xy[conf_mask]
        dets_cls = dets_cls[conf_mask]
        if self.drow_to_ros and len(dets_xy) > 0:
            dets_xy[:, 1] *= -1.0

        # Combine close detections
        combined_dets_xy, combined_dets_cls, combined_radii = combine_close_detections(
            dets_xy, dets_cls, base_radius=0.4, combine_threshold=0.4
        )

        # Transform detections only when the requested output frame differs.
        try:
            new_header = msg.header
            if self.target_frame and self.target_frame != msg.header.frame_id:
                transform_stamped = self._tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    rclpy.time.Time.from_msg(msg.header.stamp),
                )
                trans = transform_stamped.transform.translation
                rot = transform_stamped.transform.rotation
                yaw = math.atan2(
                    2.0 * (rot.w * rot.z + rot.x * rot.y),
                    1.0 - 2.0 * (rot.y * rot.y + rot.z * rot.z),
                )
                cosine, sine = math.cos(yaw), math.sin(yaw)
                if len(combined_dets_xy) > 0:
                    x = combined_dets_xy[:, 0].copy()
                    y = combined_dets_xy[:, 1].copy()
                    combined_dets_xy[:, 0] = cosine * x - sine * y + trans.x
                    combined_dets_xy[:, 1] = sine * x + cosine * y + trans.y
                new_header.frame_id = self.target_frame

        except TransformException as ex:
            self.get_logger().warn(
                f"Could not transform from {msg.header.frame_id} "
                f"to {self.target_frame}: {ex}"
            )
            # If transform fails, keep original XY and original header
            new_header = msg.header

        # Clear old markers before publishing new ones
        self._clear_old_markers(new_header)

        # Publish new detections
        dets_msg = detections_to_pose_array(combined_dets_xy, combined_dets_cls, self.get_logger())
        dets_msg.header = new_header  # Use the possibly updated header
        self._dets_pub.publish(dets_msg)

        self._scored_pub.publish(
            detections_to_scored_pointcloud(
                combined_dets_xy, combined_dets_cls, new_header
            )
        )

        rviz_msg = detections_to_rviz_marker(combined_dets_xy, combined_dets_cls, new_header, radii=combined_radii)
        # self.get_logger().info(f"Number of markers in rviz_msg: {len(rviz_msg.markers)}")
        self._rviz_pub.publish(rviz_msg)

    def _clear_old_markers(self, header):
        """Publish a DELETEALL Marker to clear previously displayed markers."""
        clear_msg = MarkerArray()
        clear_marker = Marker()
        clear_marker.header = header
        clear_marker.ns = "dr_spaam_ros"
        clear_marker.id = 0
        clear_marker.action = Marker.DELETEALL
        clear_msg.markers.append(clear_marker)
        self._rviz_pub.publish(clear_msg)


def combine_close_detections(dets_xy, dets_cls, base_radius=0.4, combine_threshold=0.1):
    if len(dets_xy) == 0:
        return dets_xy, dets_cls, []

    clusters = []
    for xy, c in zip(dets_xy, dets_cls):
        assigned = False
        for cluster in clusters:
            centroid = cluster['centroid']
            dist = np.linalg.norm(xy - centroid)
            if dist < combine_threshold:
                cluster['points'].append(xy)
                cluster['confs'].append(c)
                cluster['centroid'] = np.mean(cluster['points'], axis=0)
                assigned = True
                break
        if not assigned:
            clusters.append({
                'points': [xy],
                'confs': [c],
                'centroid': xy.copy()
            })

    combined_dets_xy = []
    combined_dets_cls = []
    combined_radii = []

    for cluster in clusters:
        pts = np.array(cluster['points'])
        confs = np.array(cluster['confs'])
        centroid = np.mean(pts, axis=0)
        combined_conf = np.max(confs)
        max_dist = np.max(np.linalg.norm(pts - centroid, axis=1))
        new_radius = base_radius + max_dist
        combined_dets_xy.append(centroid)
        combined_dets_cls.append(combined_conf)
        combined_radii.append(new_radius)

    return np.array(combined_dets_xy), np.array(combined_dets_cls), combined_radii

def detections_to_rviz_marker(dets_xy, dets_cls, header, radii=None):
    if radii is None:
        radii = [0.4]*len(dets_xy)

    markers = MarkerArray()
    for idx, ((x, y), d_cls, r) in enumerate(zip(dets_xy, dets_cls, radii)):
        circle_marker = Marker()
        circle_marker.header = header
        circle_marker.ns = "dr_spaam_ros"
        circle_marker.id = idx * 2
        circle_marker.type = Marker.LINE_STRIP
        circle_marker.action = Marker.ADD
        circle_marker.pose.orientation.w = 1.0
        circle_marker.scale.x = 0.03
        circle_marker.color.r = 1.0
        circle_marker.color.a = 1.0

        ang = np.linspace(0, 2 * np.pi, 20)
        xy_offsets = r * np.stack((np.cos(ang), np.sin(ang)), axis=1)

        for i in range(len(xy_offsets)):
            p = Point()
            p.x = float(x + xy_offsets[i, 0])
            p.y = float(y + xy_offsets[i, 1])
            p.z = 0.0
            circle_marker.points.append(p)
        # Close the circle
        circle_marker.points.append(circle_marker.points[0])
        markers.markers.append(circle_marker)

        text_marker = Marker()
        text_marker.header = header
        text_marker.ns = "dr_spaam_ros"
        text_marker.id = idx * 2 + 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = float(x)
        text_marker.pose.position.y = float(y)
        text_marker.pose.position.z = 0.5
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.3
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = "{:.2f}".format(d_cls)

        markers.markers.append(text_marker)

    return markers

def detections_to_pose_array(dets_xy, dets_cls, logger):
    pose_array = PoseArray()
    for d_xy, d_cls in zip(dets_xy, dets_cls):
        p = Pose()
        p.position.x = float(d_xy[0])
        p.position.y = float(d_xy[1])
        p.position.z = 0.0
        pose_array.poses.append(p)
    return pose_array


def detections_to_scored_pointcloud(dets_xy, dets_cls, header):
    """Return x/y/confidence triples with an explicit ROS frame and stamp."""
    message = PointCloud2()
    message.header = header
    message.height = 1
    message.width = len(dets_xy)
    message.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(
            name='confidence', offset=8, datatype=PointField.FLOAT32, count=1
        ),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = message.point_step * message.width
    message.is_dense = True
    message.data = b''.join(
        struct.pack('<fff', float(x), float(y), float(confidence))
        for (x, y), confidence in zip(dets_xy, dets_cls)
    )
    return message

def main(args=None):
    rclpy.init(args=args)
    node = DrSpaamROS()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
