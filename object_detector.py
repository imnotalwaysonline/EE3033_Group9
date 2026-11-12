#!/usr/bin/env python3
"""Consume lab YOLOv8 detections and estimate a bottle's approximate map position."""
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from std_msgs.msg import Bool
from yolo_msgs.msg import DetectionArray


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector_pixel_height')
        self.target_class = 'bottle'
        self.min_confidence = 0.60
        self.object_height_m = 0.20  # Measure your actual bottle
        self.fx = 581.31773  # Replace with calibration for current camera
        self.fy = 582.44720
        self.cx = 319.53861
        self.camera_forward_offset = -0.02  # Verify mounting geometry
        self.image_width = 640
        self.image_height = 480
        self.min_pixel_height = 15.0
        self.max_distance_m = 3.0
        self.min_distance_m = 0.15
        self.max_pose_age_s = 1.0
        self.required_detections = 3
        self.count = 0
        self.confirmed = False
        self.pose = None
        self.pose_received_at = None
        self.last_point = None
        self.last_point_at = None

        self.detection_sub = self.create_subscription(
            DetectionArray, '/yolo/detections', self.detection_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.pause_pub = self.create_publisher(Bool, '/object/pause_request', 10)
        self.confirm_pub = self.create_publisher(Bool, '/object_detected', 10)
        self.point_pub = self.create_publisher(
            PointStamped, '/object/preliminary_map_point', 10)
        self.get_logger().info('Lab YOLO detection subscriber ready; no approach motion.')

    def pose_callback(self, msg):
        self.pose = msg.pose.pose
        self.pose_received_at = time.monotonic()

    def detection_callback(self, msg):
        if self.confirmed:
            return
        objects = [d for d in msg.detections
                   if d.class_name.lower() == self.target_class
                   and float(d.score) >= self.min_confidence]
        if not objects:
            self.count = 0
            self.last_point = None
            return

        # Same strategy as previous detector: highest-confidence bottle per frame.
        det = max(objects, key=lambda d: float(d.score))
        u = float(det.bbox.center.position.x)
        v = float(det.bbox.center.position.y)
        width_px = float(det.bbox.size.x)
        height_px = float(det.bbox.size.y)
        x1, x2 = u - width_px / 2.0, u + width_px / 2.0
        y1, y2 = v - height_px / 2.0, v + height_px / 2.0
        if (not all(map(math.isfinite, (u, v, width_px, height_px)))
                or width_px <= 0 or height_px < self.min_pixel_height
                or x1 <= 1 or x2 >= self.image_width - 2
                or y1 <= 1 or y2 >= self.image_height - 2
                or self.pose is None or self.pose_received_at is None
                or time.monotonic() - self.pose_received_at > self.max_pose_age_s):
            self.count = 0
            self.get_logger().warn('Object visible, but full box / fresh AMCL unavailable.')
            return

        # Approximate pinhole-camera distance; assumes upright, fully visible bottle.
        distance = self.fy * self.object_height_m / height_px
        if not (self.min_distance_m <= distance <= self.max_distance_m):
            self.count = 0
            self.get_logger().warn(f'Estimated distance {distance:.2f} m out of range.')
            return

        forward = distance + self.camera_forward_offset
        left = -(u - self.cx) * distance / self.fx
        pose = self.pose
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                         1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        bx = pose.position.x + forward * math.cos(yaw) - left * math.sin(yaw)
        by = pose.position.y + forward * math.sin(yaw) + left * math.cos(yaw)
        if not (math.isfinite(bx) and math.isfinite(by)):
            self.count = 0
            return

        point = PointStamped()
        point.header.frame_id = 'map'
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x, point.point.y, point.point.z = bx, by, 0.0
        self.point_pub.publish(point)
        self.last_point = (bx, by)
        self.last_point_at = time.monotonic()
        if self.count == 0:
            self.pause_pub.publish(Bool(data=True))
        self.count += 1
        self.get_logger().info(
            f'Object {self.count}/{self.required_detections} | '
            f'box_height={height_px:.0f}px | distance~{distance:.2f}m | '
            f'map~({bx:.2f},{by:.2f})')
        if self.count >= self.required_detections:
            self.confirmed = True
            self.confirm_pub.publish(Bool(data=True))
            self.get_logger().warn(
                f'OBJECT CONFIRMED; approximate map point ({bx:.2f}, {by:.2f}). '
                'Requested Nav2 return via /object_detected.')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
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
