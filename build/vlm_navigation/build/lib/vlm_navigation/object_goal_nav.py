#!/usr/bin/env python3
"""Open-vocabulary, VLM-driven object-goal navigation.

Say "go to the red box" and the robot finds it: a local vision-language model
(YOLO-World) detects the named object in the live RGB-D stream and the robot
drives there. There is no hardcoded table of object locations.

Pipeline: resolve the phrase -> search (rotate in place, then visit vantage
waypoints) -> localize (back-project the detection through depth + TF into the
map frame) -> navigate to a standoff goal -> confirm with the VLM. CPU-only.

Usage:
  ros2 run vlm_navigation object_goal_nav --ros-args -p target:="red box"
  ros2 run vlm_navigation object_goal_nav --ros-args -p target:="red box" -p skip_nav:=true
"""

import math
import sys

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import tf2_ros
from tf2_ros import TransformException

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Twist, PointStamped
from cv_bridge import CvBridge
import tf2_geometry_msgs

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from vlm_navigation.vlm_detector import VLMDetector, resolve_target, TARGET_VOCAB


# Vantage waypoints (map frame) to visit if the object is not visible from the
# start pose; chosen so they collectively see every corner. Tune for your world.
SEARCH_WAYPOINTS = [
    (0.0, 0.0),
    (2.5, 2.5),
    (-2.5, -2.5),
    (2.5, -2.5),
]
ROTATE_SPEED = 0.5
ROTATE_STEP_RAD = 0.30
ROTATE_STEPS = 24

STANDOFF_M = 1.0
MIN_RANGE_M = 0.2
MAX_RANGE_M = 6.0
DEPTH_PATCH = 5


class ObjectGoalNav(Node):
    def __init__(self):
        super().__init__('object_goal_nav')
        self.declare_parameter('target', 'red box')
        self.declare_parameter('skip_nav', False)
        self.declare_parameter('search_in_place_only', False)

        self.bridge = CvBridge()
        self.rgb = None
        self.depth = None
        self.cam_k = None

        self.detector = VLMDetector(threads=0)

        self.create_subscription(Image, '/rgbd/image',
                                 self._rgb_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/rgbd/depth_image',
                                 self._depth_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/rgbd/camera_info',
                                 self._info_cb, 10)

        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _rgb_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self.rgb = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    def _depth_cb(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    def _info_cb(self, msg):
        self.cam_k = np.array(msg.k).reshape(3, 3)

    def _spin(self, secs):
        """Spin ROS for a wall-clock duration."""
        t0 = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if (self.get_clock().now() - t0).nanoseconds / 1e9 >= secs:
                return

    def wait_for_map_tf(self, timeout=20.0) -> bool:
        """Block until map->camera_optical_link is in the TF buffer.

        The listener needs to accumulate /tf messages after AMCL starts, or an
        early detection back-projects before 'map' exists in this buffer.
        """
        t0 = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.tf_buffer.can_transform(
                    'map', 'camera_optical_link', rclpy.time.Time()):
                return True
            if (self.get_clock().now() - t0).nanoseconds / 1e9 > timeout:
                return False
        return False

    def wait_for_frames(self, timeout=15.0) -> bool:
        t0 = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.rgb is not None and self.depth is not None \
                    and self.cam_k is not None:
                return True
            if (self.get_clock().now() - t0).nanoseconds / 1e9 > timeout:
                return False
        return False

    def detect_once(self, target_key):
        """Run the VLM+HSV detector on the freshest frame. Returns Detection|None."""
        for _ in range(6):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.rgb is not None:
                break
        if self.rgb is None:
            return None
        return self.detector.detect(self.rgb, target_key)

    def backproject_to_map(self, det):
        """Back-project the detection (pixel + depth) into a map-frame (x, y, range).

        Pinhole un-projection with the camera intrinsics, then a TF hop into map.
        The intrinsics are rescaled from the principal point because ros_gz
        publishes K at half the image resolution (cx=160/cy=120 for a 640x480
        image). The transform is looked up at the latest time and applied
        directly, since the message-stamp form fails on a zero stamp.
        """
        if self.depth is None or self.cam_k is None:
            self.get_logger().warn('No depth/intrinsics for back-projection.')
            return None

        h, w = self.depth.shape[:2]
        u, v = det.cx, det.cy
        k = DEPTH_PATCH
        u0, u1 = max(0, u - k), min(w, u + k + 1)
        v0, v1 = max(0, v - k), min(h, v + k + 1)
        patch = self.depth[v0:v1, u0:u1]
        valid = patch[np.isfinite(patch) & (patch > MIN_RANGE_M)
                      & (patch < MAX_RANGE_M)]
        if valid.size == 0:
            self.get_logger().warn(
                f'No valid depth at detection centroid ({u},{v}).')
            return None
        z = float(np.median(valid))

        sx = (w / 2.0) / self.cam_k[0, 2] if self.cam_k[0, 2] else 1.0
        sy = (h / 2.0) / self.cam_k[1, 2] if self.cam_k[1, 2] else 1.0
        fx = self.cam_k[0, 0] * sx; fy = self.cam_k[1, 1] * sy
        cx = self.cam_k[0, 2] * sx; cy = self.cam_k[1, 2] * sy
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        pt = PointStamped()
        pt.header.frame_id = 'camera_optical_link'
        pt.point.x = float(x)
        pt.point.y = float(y)
        pt.point.z = float(z)
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'camera_optical_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
        except TransformException as e:
            self.get_logger().warn(f'TF camera->map failed: {e}')
            return None
        mp = tf2_geometry_msgs.do_transform_point(pt, tf)
        return mp.point.x, mp.point.y, z

    def robot_xy_in_map(self):
        """Current robot position in map frame, or None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
        except TransformException as e:
            self.get_logger().warn(f'TF map->base_link failed: {e}')
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    def standoff_goal(self, obj_xy):
        """A PoseStamped STANDOFF_M short of the object, facing it."""
        ox, oy = obj_xy
        rxy = self.robot_xy_in_map()
        rx, ry = rxy if rxy else (0.0, 0.0)
        vx, vy = ox - rx, oy - ry
        dist = math.hypot(vx, vy) or 1.0
        ux, uy = vx / dist, vy / dist
        reach = max(0.0, dist - STANDOFF_M)
        gx, gy = rx + ux * reach, ry + uy * reach
        yaw = math.atan2(oy - gy, ox - gx)

        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x = gx
        p.pose.position.y = gy
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def rotate_and_scan(self, target_key):
        """Rotate ~360 in steps, detecting at each. Returns Detection|None."""
        t = Twist()
        for i in range(ROTATE_STEPS):
            det = self.detect_once(target_key)
            if det is not None:
                self.cmd.publish(Twist())
                self.get_logger().info(f'  VLM sighting at step {i}: {det}')
                return det
            dur = ROTATE_STEP_RAD / ROTATE_SPEED
            t.angular.z = ROTATE_SPEED
            t0 = self.get_clock().now()
            while (self.get_clock().now() - t0).nanoseconds / 1e9 < dur:
                self.cmd.publish(t)
                rclpy.spin_once(self, timeout_sec=0.02)
            self.cmd.publish(Twist())
            self._spin(0.3)
        return None

    def search(self, target_key, nav, in_place_only):
        """Find the target: scan here, then at each waypoint. Detection|None."""
        self.get_logger().info('Searching (rotate in place)...')
        det = self.rotate_and_scan(target_key)
        if det is not None or in_place_only:
            return det

        for wx, wy in SEARCH_WAYPOINTS:
            self.get_logger().info(f'Not found; driving to waypoint ({wx},{wy})...')
            wp = PoseStamped()
            wp.header.frame_id = 'map'
            wp.header.stamp = nav.get_clock().now().to_msg()
            wp.pose.position.x = float(wx)
            wp.pose.position.y = float(wy)
            wp.pose.orientation.w = 1.0
            nav.goToPose(wp)
            while not nav.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.1)
            if nav.getResult() != TaskResult.SUCCEEDED:
                self.get_logger().warn(f'  could not reach waypoint ({wx},{wy}).')
                continue
            det = self.rotate_and_scan(target_key)
            if det is not None:
                return det
        return None

    def run(self):
        phrase = self.get_parameter('target').value
        try:
            key = resolve_target(phrase)
        except KeyError:
            self.get_logger().error(
                f"Unknown target '{phrase}'. Known colors: {list(TARGET_VOCAB)}")
            return False
        self.get_logger().info(
            f"Object-goal (VLM): '{phrase}' -> target color '{key}'")

        self.get_logger().info('Loading VLM + waiting for RGB-D frames...')
        if not self.wait_for_frames():
            self.get_logger().error('No RGB-D frames/intrinsics; is gazebo running?')
            return False

        if self.get_parameter('skip_nav').value:
            self.get_logger().info('skip_nav=True: detecting in place...')
            det = self.detect_once(key)
            if det is None:
                self.get_logger().warn(f"✗ '{phrase}' not detected in view.")
                return False
            obj = self.backproject_to_map(det)
            self.get_logger().info(
                f"✓ Detected '{phrase}': {det}"
                + (f' -> map xy=({obj[0]:.2f},{obj[1]:.2f}) range={obj[2]:.2f}m'
                   if obj else ' (back-projection unavailable: needs map TF)'))
            return True

        nav = BasicNavigator()
        self.get_logger().info('Waiting for Nav2 to be active...')
        nav.waitUntilNav2Active()

        self.get_logger().info('Waiting for map TF (AMCL)...')
        if not self.wait_for_map_tf():
            self.get_logger().error(
                'map->camera TF never appeared; is AMCL/Nav2 publishing?')
            return False

        in_place = self.get_parameter('search_in_place_only').value
        det = self.search(key, nav, in_place)
        if det is None:
            self.get_logger().warn(f"✗ '{phrase}' not found after search.")
            return False

        obj = self.backproject_to_map(det)
        if obj is None:
            self.get_logger().error(
                'Found the object but could not localize it (depth/TF). Aborting.')
            return False
        ox, oy, rng = obj
        self.get_logger().info(
            f"Localized '{phrase}' at map ({ox:.2f},{oy:.2f}), range {rng:.2f}m.")

        goal = self.standoff_goal((ox, oy))
        goal.header.stamp = nav.get_clock().now().to_msg()
        gx, gy = goal.pose.position.x, goal.pose.position.y
        self.get_logger().info(f'Navigating to standoff pose ({gx:.2f},{gy:.2f})...')
        nav.goToPose(goal)
        while not nav.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
        if nav.getResult() != TaskResult.SUCCEEDED:
            self.get_logger().error(f'Nav2 did not reach the pose: {nav.getResult()}')
            return False

        self.get_logger().info('Reached standoff. Confirming with the VLM...')
        det2 = self.detect_once(key)
        if det2 is not None:
            self.get_logger().info(f"✓ CONFIRMED '{phrase}' in view: {det2}")
            return True
        self.get_logger().warn(
            f"✗ Reached the goal but '{phrase}' is not confirmed in view.")
        return False


def main(args=None):
    rclpy.init(args=args)
    node = ObjectGoalNav()
    try:
        ok = node.run()
    finally:
        node.cmd.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
